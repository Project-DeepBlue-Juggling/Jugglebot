# Hardware Session — 2026-07-25 self-toss anomaly fixes: bench validation

**Run**: the 2026-07-25 self-toss anomaly-fix run (four sibling plans, eleven
landed phases, one phase runner), **plus one phase added after the run closed** at
the operator's request — `hand-command-continuity` Phase 6, § Section FW, which
made the Platform Teensy flash confirmable. Each phase appended its own section as
it landed, under a strict never-reorder rule so parallel phases could not clobber
each other.

**Goal**: confirm on hardware that each landed anomaly fix does what its plan
claims, with numeric PASS/ABORT criteria and a named analysis command per check
so a failure routes straight back to the plan + phase that owns it.

> ## WHICH RUNBOOK IS THIS, AND WHEN — read this before HOW TO USE
>
> **This file is the one to step through after the build gate.** It validates that
> the eleven landed fixes do what their plans claim. It is *not* the toss
> capability ladder.
>
> | file | question it answers | when |
> |---|---|---|
> | **this file** | *did the 2026-07-25 anomaly fixes land and work?* | **FIRST**, immediately after `colcon build` + relaunch (+ the firmware flash) |
> | `session_phase8_toss_hardware.md` | *how well does the toss work?* — the T0–T4 rungs: release characterisation, vertical toss-and-catch, height ladder, toss-at-position, Tier-8b displaced | **AFTER**, once this file passes — its rungs are only meaningful on a machine whose fixes are confirmed |
> | `session_phase8_toss_trace.md` | the trace-capture recipe both files' analysis rows depend on | as referenced |
>
> Running the T-rungs first is not dangerous, but it wastes the sitting: a failure
> there cannot distinguish "the toss is imperfect" from "a fix did not land", which
> is precisely the discrimination this file exists to provide. The overlap is
> deliberate and small — this file's CAP-WORK stage already requires ≥ 12 reloads
> and ≥ 7 tosses, so it exercises T1-equivalent ground while scoring it against the
> fixes rather than against the capability.
>
> **Build gate, stated once because it has three different answers across this
> run** — the full form below is correct and strictly covers everything:
> ```bash
> cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
> source install/setup.bash
> ```
> then **relaunch** (the launch runs the *installed* copy). `jugglebot_interfaces`
> is not optional: `TrajectoryStatus.msg` gained a field and `RobotState.msg`
> gained two, and rebuilding only
> `jugglebot` makes `trajectory_node` **exit** ~200 ms after launch. Phase 4
> additionally needs a **firmware flash** of the Platform Teensy — see § THE RUN
> SHEET stage 2. **That flash is now confirmable in one line** (run-sheet row
> **FW-1**, stage 3): the board declares a `FW_VERSION` and reports it to the
> Jetson, so an un-flashed board is no longer silently indistinguishable from the
> pre-fix one.
>
> ## HOW TO USE THIS FILE — read this paragraph before anything else
>
> **§ THE RUN SHEET below is the executable document. Work down it.** The
> `## Section …` chapters after it are the *reference*: every PASS/ABORT number,
> every "why this threshold", every trap. You dive into a section when a run-sheet
> row fails or when you need the number behind a row — not while executing.
>
> The run sheet was written **2026-07-27**, after all eleven phases had landed, and
> it is the only part of this file written with the whole run in view. Where it and
> a section disagree, **the run sheet wins and the section is annotated in place**;
> nothing has been deleted. Two sections carry `⚠ SUPERSEDED` banners for exactly
> this reason (§ Section LVL and § CHECK CCATCH-3) — read the banner before the
> table under it.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command in this file.**
  The implementing sessions prepared the exact commands and criteria and verify
  read-only artefacts (logs, bags) afterwards.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.**
- E-STOP always in reach. Any ABORT criterion ⇒ stop, capture the bag and the
  node logs, and debrief before retrying.

## STANDING RULES — stated once, apply to every check in this file

Every section below used to restate these. They are here, once, so a run-sheet
row can just say "standing rules apply".

1. **~~POWER-CYCLE THE CAN-BRIDGE TEENSY before the sitting~~ — RETIRED
   2026-08-15.** Log `uptime_ms` alongside **every** timing measurement (achieved
   flight, catch error, `shift`, any inter-arrival gap) — *that* half stands. The
   power-cycle half existed only because tracking lag grew with the board's uptime
   (10 ms fresh → ~240 ms at 30 h); the root cause was the vendored FlexCAN_T4
   `_available` RX-ring leak, fixed in **FW 14** and validated at 5.8 h and 15.2 h
   of continuous uptime (`logbook/2026-08-15-fw14-validated-arc-closed.md`).
   **On FW 14+ uptime is no longer a tracking-quality variable — do not reboot the
   bridge for timing reasons.** What replaces the ceremony: keep the `uptime_ms`
   label (it is now the fix's own regression detector) and watch the
   `latency_monitor` row on `/link_status` during the sitting — it alarms on RX-ring
   leak, encoder-cache age and sustained lead-clamp duty while you are still in the
   session. Anything below FW 14 is still subject to the old rule; check
   `bridge_fw_version` on `/link_status` before assuming otherwise. Every "standing
   rules apply" reference below inherits this retirement.
2. **CHECK the correction after every launch; `level` only if it is missing.**
   *(Corrected 2026-07-27. This rule previously read "run a manual `level` after
   every launch and every relaunch". That was wrong — it would cost a needless
   levelling routine on every build gate, and it mis-states two of this file's own
   checks. The correction is recorded rather than silently swapped because the
   wrong version is the kind a future session would inherit.)*

   The correction inside `trajectory_node` **is** per-process, but it is normally
   **restored automatically on ROS2 boot**. `RobotState` carries both
   `levelling_complete` **and** `pose_offset_rad` from the **Platform** Teensy
   (decoded `teensy_bridge_node.py:349-372`, published `:1747-1755`); the
   orchestrator stores both
   (`orchestrator_node.py:165-167`) and pushes the persisted offset to
   `/gravity_offset` on the first IDLE entry after boot (`:329-335`). Standing
   rule 1's power-cycle is the **can-bridge** Teensy, which does **not** clear the
   Platform Teensy's cache — so a relaunch, including the build gate's, should come
   back already levelled.

   **Why you check rather than assume:** `/gravity_offset` is VOLATILE, so a
   `trajectory_node` that finishes subscribing after the push misses it. Whether
   discovery wins that race is **unmeasured** — this sitting is the first
   opportunity to observe it, so record what you see.

   ```bash
   ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded
   ```
   `true` ⇒ nothing to do. `false` ⇒ run `level`. It is accepted only from **IDLE**
   and returns to IDLE, so it goes *before* `activate`. A fresh **Platform Teensy**
   power-cycle clears the cache and always requires a manual `level`.

   > **⚠ THIS AFFECTS CHECK LG-1 AND CHECK LG-3, WHICH ASSUME THE OPPOSITE.** Both
   > are written as "launch/relaunch without `level` ⇒ expect
   > `REJECTED_NOT_LEVELLED`". If the boot auto-push wins the race,
   > `gravity_correction_loaded` reads `true`, the toss proceeds, and **both checks
   > fail on a healthy machine.** Before scoring either, read the flag. If it is
   > `true`, the check's precondition was never established — but the two checks
   > need **different** recipes, and conflating them is what left `LG-3` contested
   > from 2026-07-28 to 2026-08-16 *(corrected 2026-08-16, from source)*:
   >
   > - **LG-1 wants BOTH flags false.** A **Platform Teensy** power-cycle gives you
   >   that (its `RobotState` is a RAM global zero-initialised at boot), not the
   >   can-bridge power-cycle standing rule 1 used to mandate. Note the price: the
   >   Platform Teensy is on Jugglebot's 12 V/ODrive supply, so the same power
   >   event clears `is_homed` and the ODrive references — budget a re-home.
   > - **LG-3 wants `levelling_complete: true` with `gravity_correction_loaded:
   >   false`, so a power-cycle is exactly the wrong tool** — every board's
   >   power-cycle either leaves both true (can-bridge: it stores nothing) or drives
   >   the Teensy flag false (Platform: LG-1's state). The deterministic recipe is
   >   to restart **`trajectory_node` alone** after a `level`; see § CHECK LG-3.
   >
   > Record which case you got — that observation settles the race question for
   > every future sitting, and is worth more than the checks themselves.
3. **Judge every catch by eye as well as by `outcome`, everywhere in this file,
   and record one truthful outcome line per attempt.** *(Rewritten 2026-07-28.
   This rule read "The tracker still reports `MISSED` on real catches" — half of
   that is no longer true, and the wrong half is the one that would make an
   operator ignore a working verdict.)*

   The possession gate was fixed (contract **C-POSSESS-1**,
   `ros_ws/docs/ball_possession_contract.md`). What to expect now, per path:

   - **self-toss** — `outcome: CAUGHT` on a real catch. Scored offline on the
     2026-07-27 capture the gate reads **17 / 17** where it previously read
     **0 / 17**. A self-toss reading `MISSED` on a catch you watched land is now
     a **finding**, not the expected noise.
   - **reload** — ~~still `MISSED` on a real catch~~ **⚠ THIS FLIPPED 2026-08-10.**
     The bullet used to read "still `MISSED` on a real catch, and that is correct
     behaviour" — because every Ball-Butler track in the 2026-07-27 capture is a
     split track whose filter is fed by the **wrong marker**, so the tracker
     refused verdicts the operator watched land. That bullet named its own expiry
     condition — *"it changes when the tracker investigation closes, or when the
     hand sensor becomes the primary source"* — and the second one has now
     happened (`logbook/2026-08-10-sensor-truth-possession.md`, catch-robustness
     Phase 1). **A reload catch now reads `CAUGHT`**: the hand ball sensor reads
     the *cup*, so it does not care where the tracker thinks the ball was, and it
     is the primary source. The tracker's arrival error survives as REPORT-only
     corroboration on the same line. A reload catch you watched land that still
     reads `MISSED` is now a **finding**, exactly like a self-toss one.

   Row **POSS-1** is where you write the by-eye counts down beside the gate's.
4. **`run_mpc.py` must NOT be running** (sole-binder on :5557) unless a check says
   so. Only CHECK FK-4 wants it, and FK-4 is optional.
5. **Two Python environments, and mixing them up costs you the capture.** The trace
   recorder's `record` subcommand (`tests/hardware/toss_trace_recorder.py`) runs
   under **system `python3` 3.8** with `/opt/ros/foxy/setup.bash` **and**
   `ros_ws/install/setup.bash` sourced — it is an `rclpy` subscriber, which the venv
   cannot provide. Everything under `tools/probes/` runs under the **venv**
   (`source ~/Desktop/PDJ_venv/venv/bin/activate`) — numpy/mcap/casadi, which system
   python3 does not have. They are opposites; each run-sheet row names which.
   (Exception worth knowing: the recorder's `check` subcommand is pure stdlib and
   runs anywhere, including the venv. Only `record` needs ROS.)
6. **The operator runs every robot-actuating command.** E-STOP in reach. Any ABORT
   ⇒ stop, keep the bag and `~/.ros/log`, debrief before retrying.

## ⚠ DEPLOYMENT MATRIX — the single biggest foot-gun in this run

Twelve phases landed across three *different* kinds of deployment. Getting this
wrong is the most likely way to waste a sitting. **Until 2026-07-27 one of the
three failed silently; Phase 6 closed that — but it made the skip VISIBLE, not
BLOCKED, so row C still depends on you running its check.**

> **Row D added 2026-07-28** — a **fourth** kind, config regeneration. It does
> NOT belong to the 2026-07-27 run: it exists only for § SECTION SEAT-EXP, a
> separate later sitting. Rows A-C are unchanged and nothing in stages 1-7 of
> § THE RUN SHEET needs D.

| | what changed | what you must do | **how you find out you skipped it** |
|---|---|---|---|
| **A** | Python under `ros_ws/src/jugglebot/**` — §§ FK, HAND-1, HAND-2, HAND-3, LVL, CCATCH, ZSEAT, **POSS** (commits `aea7b49`, `e58ed89`, the hand phases, and the C-POSSESS-1 commit). **§ SECTION POSS adds a NEW module** (`ball_possession.py`), which is the one shape that can land half-applied from a cached build | `colcon build --packages-select jugglebot_interfaces jugglebot` + `source install/setup.bash` + **relaunch** `jugglebot_launch.py`. **Both packages** (the two-package build is mandatory in EVERY section since 2026-07-29 — `reload_coordinator_node` imports `TossContinuous` at module scope, so a `jugglebot`-only build raises `ImportError` before that node is constructed and takes `Reload`, `Toss` and `TossContinuous` down together; matrix row B) | **Loudly, if you run the pre-flights.** Each affected section has a grep against the *installed* copy that prints `PF<n>_STALE` on the run sheet (PF-1…PF-4 and **PF-7**, stage 3) and `INSTALLED_STALE` in the per-section pre-flights — two token spellings for one check, so match on the `STALE` suffix, not the whole word. Skip the pre-flight and the section silently re-measures the pre-fix baseline and you score a working fix as broken |
| **B** | `jugglebot_interfaces` — `TrajectoryStatus.msg` gained `gravity_correction_loaded` (§ Section LVLGATE, commit `e36d60d`) and, **2026-08-02, `tilt_map_loaded` + `tilt_map_version`** (contract C-LEVEL-2, `plans/archived/tilt-calibration-grid.md` Phase 2), `RobotState.msg` gained `platform_fw_version` / `platform_fw_version_read` (§ Section FW), and **2026-07-29 a whole NEW action, `TossContinuous.action`** (§ SECTION CONT) | `colcon build --packages-select jugglebot_interfaces jugglebot` + `source install/setup.bash` + **relaunch**. **Building only `jugglebot` is NOT enough** | **Loudly and catastrophically, now from two nodes.** `_publish_status` assigns a field the generated message's `__slots__` lack, raising inside the 0.2 s timer; rclpy re-raises timer exceptions out of `spin()` and `main` catches only `KeyboardInterrupt`, so **`trajectory_node` EXITS ~200 ms after launch**. You see: no `trajectory_node` in `ros2 node list`, no 40 Hz hold stream, `ros2 topic echo /trajectory/status` hangs, and **`activate` FAILS at the A2 arm ("no mpccmd frame")** — you never reach TRAJECTORY, so you never send a toss at all. LG-0 catches it in 3 s. **`teensy_bridge_node` behaves DIFFERENTLY — do not expect it to exit.** Its 100 Hz `_publish_robot_state` assigns the two new `RobotState` fields but *catches its own exceptions*, so a half-rebuild there gives you **one throttled `Robot state publish error:` per 5 s and a silently-dead `/robot_state`** — the node stays in `ros2 node list` looking healthy while the orchestrator stalls in BOOT and blames power/CAN. Since 2026-07-27 it also logs, once at construction, `INTERFACES_STALE: …` naming the missing fields and the exact rebuild command — **grep that first** (`grep INTERFACES_STALE "$LOG"`). Note this matters most when `jugglebot_interfaces` is only *partly* stale: if it already carries **every** `TrajectoryStatus` field `_publish_status` assigns, `trajectory_node` does NOT exit and the loud row-B signature above never appears. **That carve-out expired on 2026-08-02**: `tilt_map_loaded` / `tilt_map_version` were added, so an interfaces package built before that date is missing a field `_publish_status` assigns and the loud exit signature is BACK — carrying `gravity_correction_loaded` from an earlier sitting is no longer enough. **2026-08-14 moved the expiry again**: `leg_vel_limit_mmps` / `leg_acc_limit_mmps2` / `leg_jerk_limit_mmps3` were added (live session limits for the toss reach bound), so a pre-2026-08-14 interfaces build re-triggers the loud exit signature. **⚠ The NEW ACTION makes row B worse, and in a way none of the above describes.** `reload_coordinator_node` imports `TossContinuous` at module scope, so a stale `jugglebot_interfaces` raises `ImportError` before the node is constructed — and that node hosts **all three** ball-op actions. You lose `Reload` and `Toss` too, not just the session: `ros2 action list` shows none of `/jugglebot/reload`, `/jugglebot/toss`, `/jugglebot/toss_continuous`, and `ros2 node list` has no `reload_coordinator_node`. The launch log carries the `ImportError` naming `TossContinuous`. `tests/hardware/toss_trace_recorder.py record` fails the same way, with its own explicit rebuild message. Row CONT-0.3 is the 3-second check |
| **C** | `ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h` + the regenerated `Teensy_code_platform/hardware_config.h` (§ CHECK HAND-4, commit `5369fc2`; **and § CHECK HAND-7's post-release decel feedforward, 2026-07-28** — a NEW `TeensyTraj::THROW_DECEL_REFLECTED_INERTIA_KGM2` in that same header), and `Teensy_code_platform.ino`'s `FW_VERSION` identity block, now at **3** (§ Section FW) | **FLASH `Teensy_code_platform/Teensy_code_platform.ino` to the PLATFORM Teensy.** Not the can-bridge (`Teensy_code_canbridge/`), not the CatchingCone. `colcon build` does not touch it and the Jetson never executes it | **Loudly, since 2026-07-27 — read the box below.** `link_status/platform_fw_version` reads `0 (PRE-VERSIONING)` on a never-flashed board, `1` on a board still carrying only the Phase-4 prelude, `2` on one carrying the Phase-7 decel feedforward, and **`3`** on one carrying the 2026-08-18 hand end-stop correction (`HAND_MOTOR_HARD_STOP_REVS` 11.1 → 10.8 rev), and the launch log carries a `PLATFORM_FW_CHECK: FAIL` ERROR. Run-sheet row **FW-1** |
| **D** | `config/hardware_config.yaml` — `trajectory_op.catch_seat_rate_radps` (§ SECTION SEAT-EXP, added 2026-07-28). Shipped value `0.0`; **only the seat-rate A/B ever moves it** | `python config/generate_config.py` (**venv**, standing rule 5) **then** `colcon build --packages-select jugglebot_interfaces jugglebot` + **relaunch** (both packages — matrix row B). The regenerate is the step that gets skipped, and skipping it changes *nothing at all* — every consumer imports the **generated** `hardware_config.py`, not the YAML | **Only if you check, and the failure is the quiet kind.** A skipped regenerate or a skipped `colcon` leaves the machine on the previous rate, so the experiment block silently repeats the control block and the A/B reads as "no difference" — a wrong *scientific* answer, not a crash. Rows `SEAT-EXP-1` and `SEAT-EXP-3` are the three-way check (installed constant, YAML, probe self-check); `SEAT-EXP-3.2` is deliberately inverted, a self-check **FAIL** naming that one constant is the positive confirmation |

> ### ⚠ SUPERSEDED (2026-07-27): the un-flashed Platform Teensy is now DETECTABLE
>
> This box used to read *"THE HEADLINE: an un-flashed Platform Teensy is
> UNDETECTABLE from the Jetson"* and hand you a **four-link circumstantial chain**
> (right source → the header compiles → you uploaded → the board rebooted) because
> the board carried no `FW_VERSION` and nothing on the wire could tell you. **That
> chain is superseded. Do not run it — run FW-1 instead.** It was inference about
> your own actions; FW-1 is an observation of the board.
>
> The board now declares `FW_VERSION` (`Teensy_code_platform.ino`, currently **3** — 1→2
> on 2026-07-28 with § CHECK HAND-7's decel feedforward, 2→3 on 2026-08-18 with
> the hand end-stop correction, `HAND_MOTOR_HARD_STOP_REVS` 11.1 → 10.8 rev) and
> reports it in bytes 5-6 of the 0x6E0 RobotState reply it already sends. See
> `ros_ws/docs/platform_fw_version.md` for the contract; the operator-facing part
> is two facts:
>
> **1. The direct check — run-sheet row FW-1, ~3 s, with the graph up:**
> ```bash
> LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log; grep PLATFORM_FW_CHECK "$LOG"
> ros2 topic echo /link_status --once | grep -A1 platform_fw_version   # live confirmation
> ```
> The log grep is the primary: the bridge emits a `PLATFORM_FW_CHECK` line at the
> **boot** RobotState read, before you think to look, and it cannot be missed by a
> flaky `topic echo`. `OK` is logged only when the version *changes* (so one line
> per launch on a healthy board, not a reconnect flood); a **`FAIL` is logged on
> every read** — if you see several, that is one board still stale, not several
> boards.
>
> | log says / `link_status` reads | means | do |
> |---|---|---|
> | `PLATFORM_FW_CHECK: OK — … v3` / `3` | flashed, current | continue |
> | `FAIL — … v2` / `2` | flashed up to the decel feedforward, but **before the 2026-08-18 end-stop correction** — `smoothMoveMaxDuration()` is still `0.8005` s, so **H4.10 will read up to `0.8005` s and score as an unflashed board** | **ABORT stage 2** — `git pull`, re-flash. The commanded *profiles* are otherwise unchanged (`SMOOTH_MOVE_POS_CEIL_REV` held at 10.60 rev because the margin moved 0.5 → 0.2 with the base), so a v2 board is not a new hazard — it is a board that cannot be scored against this file |
> | `FAIL — … v1` / `1` | flashed, but only up to Phase 4 — **no decel feedforward** | **ABORT stage 2** — `git pull`, re-flash. § CHECK HAND-7 is meaningless on a v1 board, and HAND-7 is the section about the end stop the hand touched |
> | `FAIL — … PRE-VERSIONING …` / `0 (PRE-VERSIONING)` | the board answered and **has not been flashed** | **ABORT stage 2 — flash it, then relaunch** |
> | `FAIL — … v<other>` / a different number | flashed, but not from this tree | **ABORT** — `git pull`, re-flash |
> | `UNKNOWN` / `unknown` | no RobotState read landed at all | **NOT a stale flash, and most likely NOT a fault.** A missed boot read on a launch-only restart is a *known benign transient* (it is also why you sometimes get a surprise re-home). **Relaunch once and re-read.** Only if it repeats: investigate CAN3/relay |
> | **nothing at all** — grep prints no line, `link_status` has no `platform_fw_version` key | the running node predates the check | **ABORT** — you skipped `colcon build`. Rebuild BOTH packages, source, relaunch. Do not score an absent `FAIL` as a pass |
>
> **2. The check REPORTS; it does not protect you.** Nothing refuses a hand
> command, a toss, or a state transition on a skew — deliberately, because the
> 0x6D0 path carries the kind-3 retract that is the only un-arm mechanism the
> Teensy offers (`platform_fw_version.md` § Warn, never refuse). **You are the
> enforcement.** A `0 (PRE-VERSIONING)` reading means every § CHECK HAND-4 row
> below is meaningless, and the machine will happily let you run them anyway.
>
> **§ CHECK HAND-7 is the OPPOSITE case, and that is the good news.** Phase 7's
> decel feedforward changes the commanded torque on **every single throw**, so an
> un-flashed board is not a subtle absence — the 0.78 m and ~1.2 m `peak` rows
> simply read their pre-fix values (10.29-10.33 and 10.86-11.06 rev). If FW-1 says
> `v3` and HAND-7's peaks look pre-fix, that is a **real finding about the
> physics**, not a deployment miss. If FW-1 says `v1`, stop and re-flash: you would
> be re-running the sitting that put the hand into its end stop. `v2` carries the
> feedforward but predates the end-stop correction — re-flash that too, or H4.10
> mis-scores.
>
> Still true, and still the reason this matters: § CHECK HAND-4's fix is a *no-op
> on the clean path by design* (`smoothMoveDuration`'s `v0 == 0` branch is
> bit-identical to the historical expression), so "HAND-4 looked identical to
> HAND-1" is simultaneously the **expected PASS** and what a skipped flash looks
> like. No capture can separate them; FW-1 can. Do not try to provoke the branch
> that *is* different — that means dispatching a hand command while the hand moves
> above 6.0 rev/s, which is either the defect Phases 1–2 removed or a deliberate
> abuse of the un-arm path.
>
> Two links of the old chain remain useful as *pre-flash* gates and are kept on the
> run sheet (PF-6 / INST-4): they check the tree you are about to flash FROM.
> FW-1 checks the board you flashed TO. Record in the session log: the commit you
> flashed from, the flash time, and the FW-1 reading.

**Recommended: do change B's build, which strictly covers A.** `colcon build
--packages-select jugglebot_interfaces jugglebot` is correct for every section in
this file and removes any chance of getting the A/B distinction wrong. There is no
downside beyond ~30 s of build time.

```bash
cd ~/Desktop/Jugglebot/ros_ws
colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
# then RELAUNCH jugglebot_launch.py — the launch runs the INSTALLED copy.
```

**No section in this file needs `python config/generate_config.py`.** The generated
artefacts are committed; `git pull` is enough.

## CAPTURES — what shares one recording

Several checks are different *readings of the same capture*. Running them twice
costs a sitting and gains nothing, so this is the map. Each capture needs the
**bag AND the trace recorder** running together from the start (see § Recording).

| capture | what happens in it | checks scored from it |
|---|---|---|
| **CAP-GATE** | empty cup: `LG-1` refusal → `level` → `LG-2` accepted goal. **Stop the trace recorder at the end of this** — LG-5 requires exactly two `loaded-flips` in one trace file | LG-1, LG-2, **LG-5**, LVL-1 |
| **CAP-RELAUNCH** | relaunch → re-arm → `LG-3` refusal → `level` again | LG-3 |
| **CAP-WORK** | the working capture: `go_home` (LVL-2) → ≥ 12 Reload attempts and ≥ 7 Tosses, interleaved (you must reload to load a ball anyway) → a closing `go_to_pose` + `go_home` | FK-1, FK-2, FK-3, HAND-1, HAND-2, HAND-3, HAND-4, LVL-2, LVL-3, LVL-4, CATCH-2, CCATCH-2, CCATCH-3, CCATCH-2t, CCATCH-4, CCATCH-5, ZSEAT-2, ZSEAT-3, ZSEAT-4, LG-4 |
| **CAP-SHORT** | optional, LAST: two tosses at `throw_height_m: 0.38` | HAND-1b |
| **CAP-DECEL** | *(added 2026-07-28, next sitting)* the § CHECK HAND-7 ladder: R0→R5, climbed in order (R0 is the band-floor over-brake rung, new 2026-07-29), **stopping at the first rung that fails**. Its own trace file — do NOT fold it into CAP-WORK, because H7.3's flatness row compares tiers within one capture and a mixed capture makes the comparison unreadable | **HAND-7** (H7.0–H7.7) |

Score the reload and toss halves of CAP-WORK with **separate probe invocations**:
`--thrower ball_butler` for the reloads, `--thrower jugglebot` for the tosses.

## THE RUN SHEET

> ### ✅ EXECUTED 2026-07-27 — outcome, and what to fix before the next run
>
> Stages 1–7 were run (can-bridge Teensy rebooted; Platform Teensy **flashed** and
> `FW-1` confirmed `v1` on all six launches — that sitting predates the v2 bump), then the `session_phase8_toss_hardware.md`
> T-rungs. **Full verdicts, every measured number, and the Discussion live in
> [`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`](../../logbook/2026-07-28-anomaly-fixes-validation-sitting.md)** — read that, not this box, before scoring anything.
>
> **Outcome in one line**: the levelling frame, the levelling gate, the FK criterion
> change and the whole hand-command-continuity set **PASS**; **`ZSEAT-2` ABORTs** on
> its bounce-out arm (3 bounce-outs, all 3 consecutive) while its rate arm passes
> (13/16 = 0.8125), with attribution **INCONCLUSIVE** against a measured BB warm-up
> drift. `LG-3` is **INCONCLUSIVE** — the auto-push won **7/7** of the sitting's
> `/gravity_offset` publishes (the boot-push subset **5/5**), so its precondition was
> never established. **`LVL-2` and `CCATCH-4` were never run**: no
> `go_home` or `go_to_pose` was issued anywhere in the sitting (≈60 s of robot time
> covers both — run them together next sitting).
>
> **Two findings that change how you run this file:**
>
> 1. **`FW-1`, `FW-2` (and any `launch.log` grep for `trajectory_node` or
>    `teensy_bridge_node` output — `gravity correction set`, `seeded hold at pose`,
>    `returning to neutral`) CANNOT WORK AS WRITTEN.** Both nodes are declared
>    `output='screen'` in `jugglebot_launch.py` (`:157`, `:171`), so their stdout
>    never reaches `launch.log`. On a **correctly flashed** board `FW-1` returns
>    nothing — which this file maps to the ABORT *"you skipped colcon build"*. Use
>    the per-node logs instead, with this form (the `python3_*.log` glob matches
>    50,943 files and zsh will not word-split an unquoted `$(find …)`):
>    ```bash
>    find ~/.ros/log -maxdepth 1 -name 'python3_*.log' -newermt '<date>' -size +0 \
>         -exec grep -H PLATFORM_FW_CHECK {} \;
>    ```
>    Same trap makes **`FK-1`, `FK-3` and `LG-4` as written SELF-PASS** — `ls -t
>    ~/.ros/log/python3_*.log` matches zero files, so `… | wc -l` prints `0` and
>    reads as a clean PASS regardless of the truth.
> 2. **Start the trace recorder BEFORE the `LG-1` goal, not after the `level`.** The
>    CAP-GATE recorder was started 2 min 34 s after LG-1's refusal and 3 min 4 s
>    after the loaded-flip, so `LG-5`'s own single-file reader returned `INCOMPLETE`.
>    The invariant was recovered only because `/trajectory/status`, `/robot_state`
>    and `/gravity_offset` were all in the bag from the start of the 15:23 recording.
>    Also note `FW-1` is **not scorable from a trace file at all** —
>    `toss_trace_recorder.py::_d_link` whitelists six `link_status` keys and drops
>    `platform_fw_version`.
>
> **Three scoring corrections:**
>
> - **Use the PERSISTED offset for any capture taken after a relaunch.** The Platform
>   Teensy truncates the stored offset to **int16 milliradians** (`Teensy_code_platform.ino:430`,
>   a C cast), so the 15:33 `level` published `[0.014455, 0.002070]` but every later
>   launch restored `[0.0140, 0.0020]`. This file's instruction to use "the radians
>   `LVL-1`'s log line printed" is **wrong** for those captures, and the loss (0.0264°)
>   is half of `LVL-3`'s ±0.05° band.
> - **`CATCH-2` cannot pass on any post-2026-07-26 capture.** `catch_reach_replay.py:354`
>   pins `THROUGH_SEAT_RATE_RADPS = 0.07` and deliberately rebuilds the pre-fix plan.
>   Its `NOT-REPRODUCED` is an instrument verdict, **not a machine finding** — and it
>   blocks `CCATCH-3` and `ZSEAT-3` with it.
> - **`ZSEAT-4`'s Tier-8b rows are mutually unsatisfiable as written**: a reach that
>   starts *at* release necessarily puts motion into the second half of a symmetric
>   `release ± 0.10 s` window. Score the pre-release half `[release−0.10, release]`.
>
> **One safety action**: five **off-run-sheet** ~1.2 m tosses drove the hand's measured
> peak to `10.86–11.06 rev` — **1.2 mm** from the declared 11.1 rev limit — as pure
> position-loop coast. **No further tosses above 0.78 m** until the true stroke limit is
> pinned (**RESOLVED 2026-08-18: the hard stop is 10.8 rev** — metal contact,
> operator-measured on the sensorised hand; the three candidates 11.124 / 11.224 /
> 11.4 rev were ALL too high, and the shipped 11.1 rev guard sat 0.3 rev PAST
> metal, so the peaks below were not "near the limit" but past it) and `FLIGHT_TIME_MAX_S`
> is reconciled with it.
>
> > **⚠️ AMENDED 2026-07-28 — IT TOUCHED. The `1.2 mm` is not headroom.** The operator
> > reports, by ear and by feel at the bench, that those ~1.2 m tosses made **light
> > physical contact with the hand's end-stop**. So "as pure position-loop coast"
> > above is wrong: `1.2 mm` is the resolution limit of 100 Hz aliased telemetry that
> > was watching a *contact*, not a clearance that was nearly consumed. **Do not read
> > the remaining margin as proportional headroom for a lower toss.** If the contact
> > coincided with the `11.0506 / 11.0621 rev` peaks then the physical stop sits
> > *below* the declared `hand_motor_max_position_revs = 11.1` guard (**and the
> > 2026-08-18 measurement confirms it: the stop is 10.8 rev, so the guard was
> > never protective**) and below all
> > three candidate anchors — i.e. the guard may not be protective. Stated as an
> > inference: it is unknown *which* of the five throws was heard (peaks ranged
> > `10.860–11.062`) and whether what was contacted is a compliant bumper or the hard
> > limit. The **`0.78 m` ceiling above stands unchanged and now rests on a confirmed
> > hit rather than an inferred near-miss.** The operator's chosen remedy is a more
> > aggressive post-release hand deceleration, **not** a height cap — and that profile
> > cannot be sized until the stop position is measured. Full chain in
> > [`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`](../../logbook/2026-07-28-anomaly-fixes-validation-sitting.md)
> > § Discussion → *the hand's end-stop margin above 0.78 m* and *Operator testimony
> > and decisions*.
> >
> > **⚠️ SUPERSEDED 2026-07-29 by § CHECK HAND-7 / stage 8, for a FW_VERSION 2 board
> > ONLY.** The remedy has landed: contract **C-HAND-2**
> > (`ros_ws/docs/hand_decel_feedforward.md`) corrects the post-release deceleration
> > feedforward, which was delivering ~70 % of the torque the commanded decel
> > physically needs. The `0.78 m` ceiling is lifted **only** by climbing § CHECK
> > HAND-7's ladder in order (R1 0.60 → R2 0.78 → R3 1.00 → R4 1.20 → R5 the band
> > ceiling), on a board that FW-1 reads as **`v3`** (`v2` carries the feedforward
> > but predates the 2026-08-18 end-stop correction), stopping at the first rung
> > that fails or on any audible contact. **On a `v0` or `v1` board the ceiling
> > stands exactly as written above.** One thing the fix did NOT do, and it matters here:
> > it did not pin the stop position. The three anchors still disagree by ~9 mm and
> > the contact still suggests the stop may sit below the 11.1 rev guard —
> > **CONFIRMED 2026-08-18 at 10.8 rev** — which is
> > why every HAND-7 band is at or below **10.60 rev**, safe under all three
> > candidates, rather than sized against a margin nobody has measured.

Read-only pre-flights first; nothing actuates the robot until stage 4.
"Routes to" is where a failure goes — the plan and phase that owns it.

### Stage 1 — instrument health (at the desk, no robot, no bag, ~2 min)

An instrument validated only against the broken shape scores a working fix as a
failure and burns the sitting. All four run under the **venv**.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py   --self-check     # INST-1
python tools/probes/levelling_tilt_bag_check.py --self-check # INST-2
python tools/probes/hand_stroke_timeline.py --gate           # INST-3
python -m pytest tests/firmware/test_hand_smooth_move_xref.py \
                tests/firmware/test_platform_fw_version_xref.py \
                tests/firmware/test_hand_throw_decel_xref.py -q       # INST-4
python tools/probes/hand_decel_authority.py --self-check      # INST-6
cd ros_ws/src/jugglebot/Teensy_code_platform && pio run && cd ~/Desktop/Jugglebot   # INST-5
```

| # | PASS | ABORT | routes to | detail |
|---|---|---|---|---|
| INST-1 | `SELF-CHECK: PASS`, **10/10 `OK`**, exit 0. Case 7 must show `planner._CATCH_TILT_THROUGH_RATE_RADPS=0.0` **and** `recorded-session rate (capture record, NOT a live mirror)=0.07` | any `BAD` | `catch-reach-degenerate-overshoot` P0/P2/P3 | § CCATCH-1, § ZSEAT-1 |
| INST-2 | `SELF-CHECK: PASS`, exit 0 (scores a synthetic post-fix session PASS, a pre-fix session FAIL, an ACTIVATE-contaminated session FAIL-with-note) | anything else | `levelling-frame-contract` P1–P2 | § LVL-0 |
| INST-3 | exit 0 and **TWO** `GATE PASS` lines: `25/25 rows within tolerance` **and** `fixed-shape branch`. **Judge on the exit code and both lines, not the row count** — the count grows whenever a reference row is added and has already produced one stale runbook | `GATE FAIL`, a missing second line, or non-zero exit. `GATE UNAVAILABLE` is different — the fixture is missing; restore or regenerate it | `hand-command-continuity` P0 | § The analysis command |
| INST-4 | `passed`, with **ZERO skips** | any failure, or `passed, N skipped` — a SKIP means `g++` was absent and the only three things that read the C++ read nothing. **Do not flash on a skip** | `hand-command-continuity` P4 / P6 / P7 | § H4.0b, § H7.0a, § Section FW |
| INST-6 | `SELF-CHECK: PASS`, exit 0 — it scores a synthetic PRE-fix capture **FLAG** and a synthetic POST-fix capture **ACCEPT**, through the same `analyse()` the bench uses | any `BAD` line. This probe produces the H7.2 / H7.3 / H7.5 verdicts and § Stage 8 treats it as the ladder's sole authority; an instrument validated only against the broken shape scores a **working fix as a failure**, which routes correct work back for rework and burns the sitting | `hand-command-continuity` P7 | § H7.0b, § CHECK HAND-7 |
| INST-5 | `[SUCCESS]` — the WHOLE Platform sketch compiles and links for the Teensy 4.0 | any error ⇒ **do not open the sketch, do not flash**; the source you are about to flash does not build. Needs network on a cold PlatformIO cache (~1 min); warm it is ~10 s | `hand-command-continuity` P6 | § Section FW |

INST-5 is a **compile gate only** — `Teensy_code_platform/platformio.ini` has no
`upload_command` on purpose and physically cannot flash the board (see its
header). It does not replace the Arduino IDE flash in stage 2; it proves the
sketch builds before you get there. Before 2026-07-27 nothing in the repository
compiled this sketch at all. It drops a `.pio/` build tree inside `Teensy_code_platform/`
— gitignored, invisible to `colcon` (`setup.py` does not glob that directory),
and safe to `rm -rf`.

### Stage 2 — build and flash (deployment matrix above)

```bash
cd ~/Desktop/Jugglebot && git pull
cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
```
then **flash the Platform Teensy** (matrix row C) from the Arduino IDE as usual,
then **launch** `jugglebot_launch.py`. Confirm the flash with **FW-1** in stage 3
— that is the check, not the boot banner.

### Stage 3 — pre-flights against the INSTALLED copy (read-only, ~1 min)

PF-1…PF-5 grep the **installed** tree, not the source — a source-tree grep proves
nothing about what the launch is running. **PF-6 is the exception and greps the
SOURCE**, because `Trajectory.h` is firmware: `colcon` never copies it and the
Jetson never runs it, so the source tree is exactly what you flash from and there
is no installed copy to check. The five `grep -c` lines print, in order, `0`
(`_apply_gravity_correction` — the deleted second copy), then `1`, `3`, `2`, `1`
at the commits this run landed. **Only ZERO vs NON-ZERO is load-bearing** on the
last four — a later comment edit moves the exact counts — but the first one must be
exactly `0`.

```bash
INST=~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -q FK_STALL_CEILING_MM $INST/motion/ik_solver.py && echo PF1_OK || echo PF1_STALE
grep -q _throw_stroke_gate_ok $INST/catch_coordinator_node.py \
  && test -f $INST/motion/trajectory/hand_stroke.py && echo PF2_OK || echo PF2_STALE
grep -E '^JB_OP_HAND_CATCH_PRIME_REV|^HAND_STROKE_TOP_REV' $INST/hardware_config.py
grep -q levelling.correct_pose $INST/trajectory_node.py && echo PF4_OK || echo PF4_STALE
grep -c _apply_gravity_correction $INST/mpc_bridge_node.py
grep -c gravity_correction_loaded \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot_interfaces/share/jugglebot_interfaces/msg/TrajectoryStatus.msg
grep -c gravity_correction_loaded $INST/trajectory_node.py
grep -c NOT_LEVELLED $INST/toss_sequencer.py
grep -c 'start_vel = current_hand_velocity' \
  ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
test -f $INST/ball_possession.py \
  && grep -q GEOM_ARM_RADIUS_MM $INST/reload_coordinator_node.py \
  && echo PF7_OK || echo PF7_STALE
```

| # | PASS | ABORT | routes to |
|---|---|---|---|
| PF-1 | `PF1_OK` | `PF1_STALE` | `fk-convergence-tolerance` P1 (§ Section FK pre-flight) |
| PF-2 | `PF2_OK` | `PF2_STALE` | `hand-command-continuity` P1 (§ HAND-0) |
| PF-3 | `JB_OP_HAND_CATCH_PRIME_REV = 9.9594` **and** `HAND_STROKE_TOP_REV = 9.95940313273228` | `9.858`, or `HAND_STROKE_TOP_REV` absent | `hand-command-continuity` P3 (§ HAND-3a) |
| PF-4 | `PF4_OK` then `0` for `_apply_gravity_correction` | `PF4_STALE`, or a non-zero count (the deleted second copy is back) | `levelling-frame-contract` P1–P2 (§ LVL-0) |
| PF-5 | all three `gravity_correction_loaded` / `NOT_LEVELLED` counts **non-zero** (at the Phase-3 commit: `1`, `3`, `2` — treat the exact numbers as informational, **zero** is the failure) | any count `0` | `levelling-frame-contract` P3 (§ LG-0) |
| PF-6 | `1` hit for `start_vel = current_hand_velocity` | `0` — you are on a pre-Phase-4 tree; **do not flash it**, `git pull` first | `hand-command-continuity` P4 (§ H4.0a) |
| PF-7 | `PF7_OK` | `PF7_STALE` — `ball_possession.py` is a **new module**, so a partially-cached build is the one way this lands half-applied. Rebuild before capturing: the § SECTION POSS checks are read at *scoring* time, hours later, and a stale install makes POSS-1 score a false ABORT on a capture you can no longer retake | `catch-reach-degenerate-overshoot` follow-on B (§ SECTION POSS) |

Then, **with the graph up** (still read-only):

```bash
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log; grep PLATFORM_FW_CHECK "$LOG"   # FW-1
grep INTERFACES_STALE "$LOG"                                                     # FW-2
ros2 node list | grep trajectory_node
ros2 topic echo /trajectory/status --once | grep -E "streaming|gravity_correction_loaded"
ros2 topic echo /link_status --once | grep -A1 platform_fw_version               # FW-1 (live)
```

| # | PASS | ABORT | routes to |
|---|---|---|---|
| FW-1 | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v3`, and `link_status/platform_fw_version` = `3` | `FAIL … PRE-VERSIONING` or `0 (PRE-VERSIONING)` ⇒ **the Platform Teensy was NOT flashed** — go back to stage 2, flash, relaunch, re-run FW-1. `FAIL … v2` ⇒ flashed, but before the 2026-08-18 end-stop correction — `git pull` + re-flash (H4.10 mis-scores a v2 board). `FAIL … v<other>` ⇒ flashed from a different tree; `git pull` + re-flash. `UNKNOWN` / `unknown` ⇒ no read landed; this is usually the **known benign boot-read transient**, so **relaunch once and re-read** before investigating CAN3 (co-signature: `cold-start boot read failed after N attempts`, `cold_start_authoritative` = `0`). **No line at all** ⇒ you skipped `colcon build`; rebuild both packages and relaunch — never score an absent `FAIL` as a pass | `hand-command-continuity` P6 (§ Section FW) |
| FW-2 | **no output** (grep exits 1) — the installed `jugglebot_interfaces` carries both new `RobotState` fields | any `INTERFACES_STALE:` line ⇒ **you built `jugglebot` without `jugglebot_interfaces`.** `/robot_state` is dead (one throttled error per 5 s, node still listed) and BOOT will time out blaming power/CAN. Rebuild BOTH packages, source, relaunch | `hand-command-continuity` P6 (§ DEPLOYMENT MATRIX row B) |

**A `0 (PRE-VERSIONING)` here invalidates every § CHECK HAND-4 row** and nothing
in the software will stop you running them — the version is reported, never
enforced. See § DEPLOYMENT MATRIX row C.

- **PASS**: node listed, both keys print, `gravity_correction_loaded: false` before any `level`.
- **ABORT**: `trajectory_node` **absent** ⇒ that is the half-rebuild signature of
  deployment-matrix row B. Rebuild **both** packages and relaunch.
- **ABORT**: node present but the key missing, or the echo hangs ⇒ a different
  publisher build. Fix before continuing.

### Stage 4 — CAP-GATE: the levelling gate (**EMPTY CUP**, first actuating stage)

Start the bag **and** the trace recorder (see § Recording) before the first
command. **Do not load a ball** — if the gate fails to fire, the sequence proceeds
to a dry empty-hand toss, which is safe but is not something to discover with a
ball seated.

| # | do | PASS | ABORT | routes to |
|---|---|---|---|---|
| LG-1 | launch → home → activate → TRAJECTORY, **no `level`**, send the toss goal | `outcome: REJECTED_NOT_LEVELLED`, goal terminates **< 1.0 s** after accept, **no platform and no hand motion** | the toss proceeds with `levelling_complete: false`; or any motion during a `REJECTED_*` | `levelling-frame-contract` P3, § LG-1 |
| LVL-1 | `deactivate` → `level` | both `Gravity offset published` and `gravity correction set` appear **after the last launch**, with identical `[tilt_x, tilt_y]`. **Record them — LVL-3 needs them** | the orchestrator line appears but `trajectory_node`'s does not; or no `gravity correction set` after the last relaunch | `levelling-frame-contract` P1–P3, § LVL-1 |
| LG-2 | `activate` → `trajectory` → re-issue the **same** goal | `gravity_correction_loaded: true` and the feedback advances past `CHECKING` to `POSITIONING` | `gravity_correction_loaded: false` after a clean `level`; or `REJECTED_NOT_LEVELLED` after a clean `level` | `levelling-frame-contract` P3, § LG-2 |
| LG-5 | **stop the trace recorder now** and run § LG-5's inline reader on that jsonl | exactly **two** `loaded-flips`, first `False`; first outcome `REJECTED_NOT_LEVELLED`; `1st outcome -> flip` **positive**; `flip -> 2nd outcome` **> +0.005 s**; second outcome not `NOT_LEVELLED` | `INCOMPLETE`, a never-happening flip, or `True` before the `level` | `levelling-frame-contract` P3, § LG-5 |

`REJECTED_WRONG_MODE` anywhere here is **not a verdict** — you are not in
ACTIVE:TRAJECTORY. Re-arm and re-issue; nothing has been tested yet.

### Stage 5 — CAP-RELAUNCH: the Teensy flag lies, the gate does not (**EMPTY CUP**)

Relaunch `jugglebot_launch.py` **without re-levelling**, then `activate` →
`trajectory` → read both flags → send the same goal.

| # | PASS | ABORT | routes to |
|---|---|---|---|
| LG-3 | `levelling_complete: true` **and** `gravity_correction_loaded: false` **and** `outcome: REJECTED_NOT_LEVELLED` — all three together are the whole point | `levelling_complete: true` and the toss **proceeds**: the gate is wired to the Teensy flag, the exact false assurance this phase removes. Stop and report | `levelling-frame-contract` P3, § LG-3 |

Then `level` again (standing rule 2) and confirm the refusal clears.

### Stage 6 — CAP-WORK: the working capture (the bulk of the sitting)

Fresh bag **and** fresh trace recorder. Sequence: `activate` → `trajectory` →
`go_home` (**LVL-2**) → then alternate Reload (loads a ball) and Toss until you
have **≥ 12 reload attempts** and **≥ 7 tosses** (≥ 5 at `throw_height_m: 0.6`,
≥ 2 at `0.78` for like-for-like against the § Pre-fix baseline table) → close with a
`go_to_pose` and a `go_home` (**CCATCH-4**, **CCATCH-5**).

```bash
# LVL-2 / CCATCH-5 opener
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
ros2 topic echo /trajectory/diagnostics --once      # note peak_leg_* and move_seq

# load a ball (this IS a reload attempt — score it for ZSEAT-2)
ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload \
  "{throw_delay_s: 0.0, catch_vel_scale: 0.0}" --feedback

# toss (≥ 5 at 0.6, ≥ 2 at 0.78)
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```

**Leave `catch/vel_scale` at its 0.9 default all sitting** (0.8 until the toss-tier
8a tuning). It multiplies the armed
event velocity and the arm window's right edge is `0.404 / v_armed`, so a *low*
scale lengthens the required lead and closes the window on a perfectly healthy
tracker. Swept against the production velocities: at a 0.55–0.56 s flight, **0.45
closes it (−15 ms)** and 0.50 barely opens it (+18 ms), while 0.9 gives +116 ms; at
0.80 s and above the window stays open across the whole shipped `[0.3, 1.5]` range.
**At the DERIVED band floor (0.4949 s, contract C-HAND-3) the knob is far tighter:
0.659 closes it**, so the 0.9 default carries only 1.36× of headroom there, against
1.78× at the retired 0.55 s floor.
Read `catch/vel_scale` **first** before routing any H1.4 `window CLOSED` warning to
a tracker fault.

**This run sheet flashes before any capture, so there is NO pre-flash control.**
That is deliberate — the flash is silent if skipped (deployment matrix row C), so
running the whole sitting in one build state removes the biggest way to waste it.
The cost is that § CHECK HAND-4's row **H4.2** ("`peak` within ±0.05 rev of the
pre-flash reading") has nothing to compare against. Use the absolute bounds instead:
row 3 of § PASS / ABORT per throw (`peak <= 10.060` rev) and H4.4's
`10.060 < peak <= 10.60` ABORT band. If you would rather have the control, capture
HAND-1 *before* stage 2's flash and take a second capture after — it costs one extra
set of tosses and the pair is more informative than either alone.

Everything below is scored **off this one capture**. Order of *scoring*, not of
running:

| # | what it scores | PASS | ABORT | routes to |
|---|---|---|---|---|
| LVL-2 | the first `go_home` after `level` is a real, small, smooth move | **SCALE THE BAND TO YOUR MEASURED OFFSET FIRST** (see below): `predicted_mm ≈ 203 × hypot(tilt_x, tilt_y)` using the **radians** LVL-1's log line printed (equivalently 3.55 mm per **degree** of total tilt — but LVL-1 gives you radians, so use the 203 form and do not convert). PASS = worst-leg excursion within **±11 %** of `predicted_mm`, realized peak leg vel within **±19 %** of `0.94 × predicted_mm` mm/s (0.94 is `1.875/T` for the 2.0 s min-jerk `go_home`; ±19 % reproduces the familiar ±0.50 mm/s at the reference offset), no pump rejection, no guard latch. At the 2026-07-25 offset (0.78185°) that is the familiar **2.77 ± 0.30 mm** / **2.60 ± 0.50 mm/s**. A **zero** measured offset makes this a genuine no-op — record which case applied | worst leg above **max(1.8 × `predicted_mm`, 2.0 mm)** (operator-set floor, 2026-07-27 — see § CHECK LVL-2's banner for what the two arms each catch), any step rejection, any `MAX_DEVIATION` or guard E-STOP | `levelling-frame-contract` P1–P2, § LVL-2 |
| CCATCH-5 | `peak_leg_*` clears on a report-less install | after `go_to_pose`: `peak_leg_vel_mmps > 0`. After `hold`/`go_home`: `move_seq` advanced **and** `peak_leg_*` all `0.0` | the previous move's non-zero peaks under the new `move_seq` | `catch-reach-degenerate-overshoot` P2, § CCATCH-5 |
| FK-1 | no spurious FK refusals | `0` × `seed FK failed`, `0` × `guard descent FK failed` across every node log | `>= 1` of either. (`non-finite target extensions` ⇒ **REPORT**, route to the can-bridge, not here) | `fk-convergence-tolerance` P1, § FK-1 |
| FK-2 | the offline FK verdict | `VERDICT: PASS`, exit 0 — `def_rai` **0** on both topics **and** `hist_rai > 0` on at least one | `def_rai > 0`. `VERDICT: VACUOUS` is **not a pass** — re-run on a richer session | `fk-convergence-tolerance` P1, § FK-2 |
| FK-3 | the FK fix is invisible in the commanded stream | every seeded hold pose matches the pre-change print **to the last digit**; `x`, `y` within **±2.0 mm** of 0.0; `max_it <= 10` | any printed digit differs (`>= 0.1 mm`), or `max_it > 10` (worst measured is 5) | `fk-convergence-tolerance` P1, § FK-3 |
| HAND-1 | the catch arm no longer lands inside the throw stroke | rows 1–5 of § PASS / ABORT per throw on **every** toss, plus H1.2–H1.7. Rows 8–9 (`stroke_end`, `post_stroke_cmd`) are REPORT — record them; row 1's ABORT cell cross-reads them | any row ABORTs — **EXCEPT row 4 (`dip_below_x3`) on a toss where row 7 (`first_neg_cmd`) is annotated**: a braking prelude fired, the two rows score the same event in opposite directions, and row 4 becomes **REPORT** (score it against the brake's own turning point — see HAND-4). `Not enough time for smooth-move` on the Teensy serial is a **hard section abort** | `hand-command-continuity` P1, § HAND-1 |
| HAND-2 | a repack under a failed ack does not clobber a live stroke | `arms` is 1 or 2 (never ≥ 3); `seeds = 0` on every `arms == 2` toss | `seeds >= 1` on an `arms == 2` toss. If **no** toss reads `arms == 2`, say so — the criterion was never exercised | `hand-command-continuity` P2, § HAND-2 |
| HAND-3 | the hand parks at the derived stroke top and nothing misjudges it | H3.1–H3.7 as tabulated; prime `pos_meas` inside **[9.4594, 10.4594]**, `Hand primed to 9.959 rev`, peak prime `vel_meas` **≤ 30 rev/s** against a commanded quintic peak of **24.63 rev/s** | outside the near-band; `9.858 rev` in the log (stale install); `>= 40.0 rev/s` | `hand-command-continuity` P3, § HAND-3 |
| HAND-4 | the flash did not break the clean path | identical to HAND-1 (**that is the designed PASS**), `peak <= 10.060` rev (no pre-flash control on this run sheet — see above), no commanded `pos < 0.0` rev, no commanded move longer than **0.8005 s**. `first_neg_cmd` annotated `<-- NOT the catch descent (a brake?)` is **REPORT, not abort** — and on such a toss score `dip_below_x3` against the brake's own turning point, not against `x3` | H4.4 / H4.5 / H4.7 / H4.8 / H4.9 / H4.10 as tabulated. **`peak > 10.60` rev is a HARD ABORT + E-STOP** | `hand-command-continuity` P4, § HAND-4 |
| LVL-3 | **the levelling headline** — the frames agree across a goal | `VERDICT: PASS`: the **park** plateau within **±0.05°** of `(−tilt_x, −tilt_y)` on both axes. **`--t0` is not optional, and its unit is SECONDS FROM BAG START** (a float, not wall-clock, not ROS epoch). Run once *without* `--t0`, read the plateau table's `t_start` column, then re-run with `--t0` a few seconds past the LVL-2 `go_home` plateau | `park_rx ≈ 0.0000°` with no `NOTE:` (pre-fix frame); or ≈ **−1.5576°** (twice the correction — applied twice) | `levelling-frame-contract` P1–P2, § LVL-3 |
| LVL-4 | mocap cross-check (does not share the FK path) | **REPORT-ONLY since 2026-07-27 — it is no longer a gate, and the old `±0.10°` PASS is the PRE-fix reading.** Run § LVL-4's inline reader and record the parked Platform-vs-`Base` tilt; expect it to have moved BY the correction, to **≈ 0.78°** (pre-fix baseline **0.087°**) | nothing here aborts on its own. `≈ 1.56°` (twice the correction) is worth stopping for — **confirm on LVL-3 first**, which is the instrumented, gated version of the same question | `levelling-frame-contract` P1–P2, § LVL-4 |
| CCATCH-2 | **the catch-reach headline** — a level catch commands NO swing | commanded `rx` across the pre-tilt reach **monotone** toward the target, peak above park ≤ `1.05 ×` the requested displacement; toss settle `rx`/`ry` = the target to **±0.05°**; residual vs gravity at contact **≤ 0.05°**; plan segments **2**; `peak_leg_acc/jerk` **≈ 1.2 / ≈ 3** (was `142.4 / 3950`) | any excursion **away** from the target > `0.05°`; settle at `−1.0784 / −0.0958°` (the old aim is live); 3 segments; still `≈142 / ≈3950` | `catch-reach-degenerate-overshoot` P2, § CCATCH-2 |
| CCATCH-2t | tracker catch error on a **self-toss** | **< 10 mm** | ≥ 16 mm — the improvement did not land. **Judge by eye too** (standing rule 3) | `catch-reach-degenerate-overshoot` P2, § CCATCH-2 |
| POSS-1 | **the possession verdict, against your own eyes** — the gate was structurally always-False until 2026-07-28. **⚠ REWRITTEN 2026-08-10: the hand ball sensor is now the PRIMARY source, which INVERTS the reload half of this row** (`logbook/2026-08-10-sensor-truth-possession.md`) | **self-toss**: gate `CAUGHT` count **== by-eye catch count**, and `>= 6/7` of the tosses run. **reload**: gate `CAUGHT` count **== by-eye reload catch count** — the old criterion (*"count 0, and every attempt logs one `possession REFUSED` — that IS the pass"*) is exactly backwards now; a reload reading `CAUGHT` is the headline capability of the change. **Zero `SENSOR_BLIND` in any possession line's `[reason]` bracket.** ⚠ **RE-READ 2026-08-26 (owner decision D1, corrected by the same day's audit):** the merge no longer falls back to the tracker, so a blind sensor shows up in the `SENSOR_BLIND` reason string and in a cycle terminal of `MISSED_SENSOR_BLIND` — **score those two, and score `MISSED_SENSOR_BLIND` first: it is the loudest and the only one that names the fault.** ⛔ **A `possession UNKNOWN` line is NOT a blindness indicator and must not be scored as one.** D1 made the question tick-driven, so UNKNOWN is now what EVERY healthy pre-arrival tick prints — a 100 % false-positive criterion, ~25 lines per cycle. The line itself now tells the two apart in words ("the sensor COULD NOT LOOK across the arrival window" vs "the arrival window has not closed yet"), so read the wording, not the verdict. The parenthetical this row carried until 2026-08-26 — *"the merge falls back to the tracker and the line names the tracker as author"* — is false in both halves. Three bags, 203,922 samples, 100 % valid, so real blindness has never been observed. The tick-driven line is `Cup: possession …` (not `Ball N:` — there is no ball to number on a tick before the arrival); the ball-shaped `Ball N: possession …` line is still one INFO line per (ball, verdict) where a tracker estimate rode along | gate count **< by-eye count − 1** on **either** path (a real catch scored MISSED); a `possession CONFIRMED` on a ball that **never arrived at the cup** — on either path, and it now means the *cup sensor* is lying, which outranks everything else in this section; **any `SENSOR_BLIND`** ⇒ record it, first hardware exercise of that path (route to POSS-1.8). **NOT an abort**: a reload reading `CAUGHT` (that is the fix working — it was the ABORT until 2026-08-10); a `CONFIRMED` on a ball that arrived and *then* left within the goal, still REPORT row `POSS-1.2b` | § SECTION POSS |
| ZSEAT-2 | **the one genuinely open experiment** — did removing the seat from an 11.08° tilted rim cost catches? | catch **RATE ≥ 0.63** (≥ 8/12, ≥ 12/19 — score the rate, the sitting only mandates `n ≥ 12`); **bounce-outs ≤ 1** across the sitting; commanded tilt over the last 0.8 s before landing **flat, < 0.05°** | rate **≤ 0.58** (≤ 6/12, ≤ 11/19); **≥ 3 bounce-outs or ≥ 2 consecutive**; `≈ 0.9°` of round trip in the last 0.8 s (stale install). **A rate strictly between 0.58 and 0.63 is INCONCLUSIVE, not a failure** — report the raw count and extend toward `n = 19` before deciding | this section — § ZSEAT-2. **Not** C-CATCH-1, **not** the levelling contract |
| ZSEAT-3 | the capture agrees with the offline counterfactual | arrival tilt rate at contact **`0.000000`**; reload settle `+1.774062 / −10.636334°` (= the target) to **±0.02°**; segments **2**; predicted `29.0 / 37.9 / 170` | non-zero arrival rate; \|settle − target\| > 0.02°; 3 segments; still `142.0 / 3935` | `catch-reach-degenerate-overshoot` P3, § ZSEAT-3 |
| ZSEAT-4 | the throw is stationary at release | commanded pose over `release ± 0.10 s` **flat**: `< 0.02°` and `< 0.2 mm` | any commanded motion — a plan is running through the release | `catch-reach-degenerate-overshoot` P3, § ZSEAT-4 |
| CCATCH-3 | the reload path changed **on purpose**, and nothing larger | seat aim rotation `4.0997°` (ABORT > 6°); the `N further catch install(s)` census line present with `N` between 6 and 25 | aim rotation > 6°. **READ THE SUPERSEDED BANNER at the head of § CCATCH-3 first — five of its rows now ABORT on correct behaviour** | `catch-reach-degenerate-overshoot` P2/P3, § CCATCH-3 |
| CCATCH-4 | blast radius really is the catch path | the closing `go_home` and `go_to_pose` behave exactly as LVL-2; no new `last_rejection` | any new rejection code, or a duration change > 5 % | `catch-reach-degenerate-overshoot` P2, § CCATCH-4 |
| CATCH-2 | the replay reproduces what the machine actually ran | `VERDICT: REPRODUCED`, exit 0; rx **and** ry max residual under the printed tolerance; fitted echo lag `0 < lag <= 25.0` ms; `SINGLE INSTALL: True`; `fk_failures 0` | `NOT-REPRODUCED` (a **publishable result**, not a probe bug — but run INST-1 first) | `catch-reach-degenerate-overshoot` P0, § CATCH-2 |
| LG-4 | no spurious levelling refusals across the sitting | `0` `Toss REJECTED_NOT_LEVELLED` on any goal issued **after** a `level` (the LG-1/LG-3 ones are the deliberate successes — subtract them) | any refusal after a `level` with `gravity_correction_loaded: true` at the time ⇒ a staleness false positive; measure the `/trajectory/status` gap **before** changing the constant | `levelling-frame-contract` P3, § LG-4 |

#### The analysis commands for stage 6, in one place

```bash
# --- under the VENV (standing rule 5) ---
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
BAG=~/Desktop/rosbags/<CAP-WORK stamp>
TRACE=temp/logs/toss_trace_<CAP-WORK stamp>.jsonl

python tools/probes/fk_convergence_bag_check.py --bag $BAG --json     # FK-2  (~92 s)
python tools/probes/hand_stroke_timeline.py --trace $TRACE --json     # HAND-1..4, H3.1/3.5/3.7
python tools/probes/levelling_tilt_bag_check.py --offset <TILT_X> <TILT_Y> \
       --t0 <SECONDS FROM BAG START, a few s after the LVL-2 go_home> \
       --bag $BAG --json                                              # LVL-3
python tools/probes/catch_reach_replay.py --bag $BAG --list           # index the tosses
python tools/probes/catch_reach_replay.py --bag $BAG --toss N --json --csv          # CCATCH-2, CATCH-2
python tools/probes/catch_reach_replay.py --bag $BAG --thrower ball_butler \
       --toss N --json                                               # CCATCH-3, ZSEAT-3
python tools/probes/possession_verdict_bag_check.py --bag $BAG        # POSS-1

# --- plain greps of the node logs (no venv needed) ---
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -c "seed FK failed"          # FK-1
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "seeded hold at pose"     # FK-3
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep -c "hand stroke-busy window latched" "$LOG"                                    # H1.2
grep -nE "hand catch arm withheld|Arming hand catch" "$LOG"                         # H1.3 / H1.7
grep "Hand primed to" "$LOG"                                                        # H3.2
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "Toss REJECTED_NOT_LEVELLED" | wc -l  # LG-4
```

`<TILT_X> <TILT_Y>` are the **radians** from LVL-1's log line.

**The three rows the block above does NOT produce, and what does** (corrected
2026-07-27 — each previously said "by eye" for a numeric criterion):

| row | quantity | the command that produces it |
|---|---|---|
| LVL-4 | parked Platform-vs-`Base` mocap tilt | § CHECK LVL-4's **inline `/rigid_body_poses` reader** — no `tools/probes/` script reads that topic. **REPORT-only** |
| ZSEAT-4 | commanded pose flat over `release ± 0.10 s` | `levelling_tilt_bag_check.py --t0 <release−0.10> --t1 <release+0.10> --plateau-min-s 0.05 --plateau-tol 1.0` (seconds from bag start) and read the plateau table's **`span_deg`** for the tilt and the **`commanded position span (x,y,z) mm`** line for the mm half, plus `samples` > 0 and `FK failures 0`. **Ignore its `VERDICT:` line** — that gate compares the *park* plateau to the offset and is meaningless on a 0.2 s window |
| ZSEAT-2 (flatness) | commanded tilt flat over the last 0.8 s before landing | same command, `--t0 <landing−0.8> --t1 <landing> --plateau-min-s 0.1 --plateau-tol 1.0`; read **`span_deg`** |

`catch_reach_replay --csv` cannot serve those last two: its series is trimmed to
the span where the *model* is defined (the plan), so it stops at the plan end —
verified 2026-07-27, `--thrower ball_butler --toss 2 --post 1.5` still ends at
`t_rel_release +0.026 s`. Raising `--post` does not extend it.

#### The `--t0/--t1` origin note — read this before running either ZSEAT row

**`--t0/--t1` are seconds from the first `/leg_setpoint_echo` sample in the bag**
(`levelling_tilt_bag_check.py` zero-bases its axis on that sample). They are **not**
seconds from the ROS epoch, and they are **not** the `rel_first_s` column that
`catch_reach_replay.py --list` prints — that column is relative to the *first
announcement's release*, a different origin, and using it directly will window the
wrong part of the bag with no error.

**Convert from the absolute release time. This is the only reliable method.**

1. Get the bag's origin — the log time of its first `/leg_setpoint_echo` sample:
   ```bash
   source ~/Desktop/PDJ_venv/venv/bin/activate
   python - <<'PY'
   import glob, sys
   from mcap_ros2.reader import read_ros2_messages
   BAG = '<bag dir>'            # e.g. ~/Desktop/rosbags/2026-07-27_14-31-02
   for p in sorted(glob.glob(BAG + '/*.mcap')):
       for m in read_ros2_messages(p, topics=['/leg_setpoint_echo']):
           print(f"bag origin = {m.log_time_ns*1e-9:.6f}"); sys.exit()
   PY
   ```
2. Get `release_abs` (a ROS epoch, 6 dp) from
   `python tools/probes/catch_reach_replay.py --bag <bag> --list`.
3. `--t0 = release_abs − origin − 0.10`, `--t1 = release_abs − origin + 0.10`
   (use `landing_abs − origin − 0.8` / `− origin` for ZSEAT-2's flatness row).

**Do NOT substitute `ros2 bag info`'s start time for the origin.**
`teensy_bridge_node` publishes `/leg_setpoint_echo` only once the pump accepts a
setpoint frame, and stops after 0.5 s without one. The bag is started *before*
`activate`, so the topic's first sample lags bag start by however long arming
takes — tens of seconds. That lag is the entire reason this note exists.

**If the probe prints `no usable /leg_setpoint_echo vectors (legacy sqlite3 bag,
or topic not recorded)`, check your WINDOW before you re-record.** That message is
also what an empty `--t0/--t1` span produces — `reconstruct` returns `None` either
way — and the topic is silent whenever the setpoint stream is stopped. The
recording is usually fine; the window missed it.

**Do NOT locate the release instant from the plateau table's edges on a toss.**
It is tempting and it produces a **false PASS**. At release the platform is in
`go_to_pose`'s terminal hold (§ CHECK ZSEAT-4, Tier 8a) — the park plateau
*contains* release, it does not end there, and it runs on until the next
operator-issued goal. Anchoring a 0.2 s window on its boundary lands you in the
idle hold, where the pose is flat by construction, and ZSEAT-4 scores PASS having
never looked at the release instant. (§ CHECK LVL-3's two-pass plateau recipe is
sound for what LVL-3 does — locating the *park itself* on a reload, at the default
`--plateau-tol 0.02`. It does not transfer to locating an *event* inside a hold.)

Do **not** mix the two conventions in one command.

### Stage 7 — CAP-SHORT (optional, LAST, only if HAND-1 passed)

Two tosses at `throw_height_m: 0.38` (T ≈ 0.557 s, just inside the band). The
suppression window narrows to **115 ms** there, against 395 ms at 0.80 s — this is
the corner where H1.4 is most likely to fire, which is the point of running it.
Score H1.1–H1.7 only. **Run it at the default `catch/vel_scale` (0.9)**: a reduced
scale closes the window at this flight length by itself and you would be measuring
the knob, not the gate. A MISSED catch here is not by itself a Phase-1 failure.

### Stage 8 — CAP-DECEL: the deceleration ladder (added 2026-07-28, ~15 min)

**Four prerequisites, and all four are cheap. Three of them run BEFORE the
flash:**

1. **H7.0a** — `pytest tests/firmware/test_hand_throw_decel_xref.py -q` passes
   with **ZERO skips**, on a host with `g++`. That file is the only thing in the
   repo that reads the C++ you are about to flash. A SKIP is not a PASS.
2. **H7.0b / INST-6** — `python tools/probes/hand_decel_authority.py --self-check`
   prints `SELF-CHECK: PASS`. This stage's verdicts all come from that probe.
3. **H7.0c** — read `axis0.config.torque_soft_min` off the **live hand ODrive**.
   If it is `-0.0551` N·m the drive will truncate the corrected feedforward and
   the whole ladder is meaningless. 30 seconds; currently an open question.
4. **A Platform Teensy on `FW_VERSION` 3** (§ DEPLOYMENT MATRIX row C). Confirm
   with **FW-1** / **H7.0** before the first toss — on a `v1` board this stage
   re-runs the sitting that put the hand into its end stop, and produces
   plausible-looking rows while doing it.

This is the only stage that goes above 0.78 m, and it is the reason the
2026-07-27 sitting's ACTION item ("no further tosses above 0.78 m") can be
lifted. Climb § CHECK HAND-7's ladder **R0 → R5, in order**, on a fresh trace,
stopping at the first rung whose `peak` leaves its band or on **any audible
end-stop contact** (H7.1). R0 is new and comes first: it is the only rung that
tests the *over*-braking direction, which bites at the bottom of the band, not
the top. R5 — the shipped band ceiling, `flight_time_s = 1.10` — runs **only** if
R0–R4 passed *and* the flatness row H7.3 passed.

```bash
# fresh recorder for this stage (system python3 + ROS env, NOT the venv)
python3 tests/hardware/toss_trace_recorder.py record
# ... climb the ladder ...
# then, under the VENV:
TR=$(ls -t temp/logs/toss_trace_*.jsonl | head -1)
python tools/probes/hand_decel_authority.py --trace "$TR" --json
# or, from the bag the recording command actually produces:
python tools/probes/hand_decel_authority.py --bag ~/Desktop/rosbags/<stamp> --json
python tools/probes/hand_stroke_timeline.py --trace "$TR" --json
```

Score with § CHECK HAND-7's tables. **H7.2 (`peak`) and H7.3 (flatness) are the
rows that decide whether the fix worked** — they read the kinematics, which are
robust. H7.4 (`dip_below_x3`) is the row that says *stop and hand it back*: it is
the over-braking direction, and it is why R0 is on the ladder at all. H7.5
(decel-phase current) is a **one-sided upper bound only** — `iq_meas` is aliased
to 0–1 fresh samples inside a 53–93 ms decel ramp, so a flat reading proves
nothing in either direction. Read the box under § CHECK HAND-7's PASS/ABORT table
before drawing any conclusion from a current column.

### Not on the run sheet, deliberately

- **CHECK FK-4** (MPC hot loop) — needs `run_mpc.py`, which standing rule 4 keeps
  off. Run it only if you exercise the MPC this sitting; otherwise record it as
  skipped.
- **CHECK CATCH-1 / CATCH-3** — superseded by INST-1 and by § CCATCH-3's banner
  respectively; CATCH-3's table is a **pre-fix reference**, not a pass criterion.
  Kept in place for the numbers.
- **CHECK LVL-5** (catch error) — **superseded by CCATCH-2t**, which is on the run
  sheet and is *gated* where LVL-5 was REPORT-only. LVL-5's `≈16 mm` was the
  through-seat residual C-CATCH-1 removed; its section carries a banner saying so.
  Nothing is dropped — the measurement still happens, under a different id and a
  tighter criterion.
- **§ Section CATCH's "When Phase 2's sitting happens"** — that sitting is this
  one; its five bullets are the standing rules plus stage 6.
- **H2.4 / H3.6 / H4.8** (SAFE_ABORT behaviour) — scored **only if a SAFE_ABORT
  occurs naturally**. Do not provoke one: a kind-3 retract clobbering an armed
  kind-0 is the only un-arm mechanism the Teensy offers.

## RESIDUAL RISK — what this run did NOT close

Read this before the sitting. None of it blocks the sitting; all of it changes how
you interpret a surprise.

1. **The reload-seat deflection risk is OPEN and this sitting is the experiment.**
   `_CATCH_TILT_THROUGH_RATE_RADPS` went to `0.0` on an operator decision, so the
   reload's 11.08°-tilted rim is now **stationary at ball contact** — exactly the
   geometry the bb-sim finding says deflects a ball. Two facts bound it without
   removing it: the `0.07` it replaced was **never hardware-validated** (its own
   leg-velocity sizing note was 2× wrong until measured), and until commit
   `407154f` the seat was aimed off the **plan-frame** tilt, so no bench impression
   was ever formed on a correctly-aimed one. **ZSEAT-2 exists to score this**, and
   its ABORT path is a one-line default change, not a redesign.
2. **An armed hand stroke produces NO observable until its event time.** The arm
   itself cannot be telemetry-confirmed the way the hand ladders were (`4e33b53`).
   HAND-2's H2.2 verifies the half a capture *can* see — a repack that clobbers a
   **live** stroke shows up as a from-rest `seed`. The other half is unobservable
   from the Jetson. A Teensy-side "armed stroke" field in `hand_telemetry` or
   `link_status` would fix it and is a protocol change, out of scope here.
3. **~~The Platform Teensy has no `FW_VERSION`~~ — CLOSED 2026-07-27.** The board
   now declares one and reports it in the 0x6E0 RobotState reply; run-sheet row
   **FW-1** reads it, and *every* future Platform-Teensy change inherits the check
   rather than the blind spot. **What remains residual is the ENFORCEMENT**: a skew
   warns and never refuses (`ros_ws/docs/platform_fw_version.md` § Warn, never
   refuse — the 0x6D0 path carries the kind-3 retract, the only un-arm mechanism
   the Teensy offers). So the operator is still the gate; the difference is that
   the operator can now see the thing they are gating on. Whether a *toss start*
   should refuse on a definite `version == 0` is open, and wants its own decision.
4. **The `ERR_TIMEOUT` hand-ack epidemic is unfixed** (`HAND_TRAJ_CMD` acks fail
   40–60 % of the time and lie in *both* directions). It is the reason the arm can
   be dispatched twice per ball, which is what HAND-2 exists to bound. More hand
   dispatches = more exposure; log dispatch-failure WARNs and any
   `ABORTED_NO_RELEASE` as epidemic gauge data. It has its own investigation.
5. **The kind-3 clobber gap is deliberately UNTOUCHED — operator's call.** Phase 4
   *narrowed* it (the empty-trajectory branch now requires the hand to be at rest
   **as well as** at the target, which strictly shrinks the condition
   `Teensy_code_platform.ino:581-583` checks before `packedMsgs.clear()` at `:588`), but the gap itself
   remains, and a pre-release SAFE_ABORT still depends on a kind-3 retract being
   able to clobber an armed stroke. H2.4 / H3.6 / H4.8 watch it; none of them
   provokes it.
6. **A velocity-continuous prelude is LONGER than the rest-to-rest one** — 0.24 s at
   the 6.0 rev/s dead-band edge, 0.32 s at 8 rev/s, against the 76 ms
   `PRELUDE_ALLOWANCE_S` budget. So at `FLIGHT_TIME_MIN_S` the firmware's own
   `Teensy_code_platform.ino:642`
   fit check could refuse the arm — a lost catch with the live stroke intact. It is
   reachable only if the hand is drifting 6–9 rev/s when a kind-1 arm lands, against
   a measured settle tail of ≤ 0.25 rev/s, so it has never been observed. Row
   **H4.7** watches for it; contract limit **F.1** records it.
7. **An operator decision this run deliberately did NOT take, with its numbers.**
   Phase 4 found the plan's premise physically unshippable: arresting `v0` costs
   `0.0077832·v0²` rev of travel, so velocity continuity is affordable only to
   **~9.1 rev/s** at the stroke top and **~19.96 rev/s** mid-stroke (the excursion
   bound; the 0.78964 s duration cap sits just above it at ~20.04 rev/s), while
   the hand passes release at **~120 rev/s** — needing **111 rev** of arrest
   travel against **10.8 rev** of stroke. (This read 20.9 / 20.3 / 0.8005 / 11.1,
   with the duration cap binding FIRST, until the 2026-08-18 hard-stop correction
   shortened the stroke and flipped the ordering.) Both plan options were rejected with numbers: refusing the command breaks
   the kind-3 clobber (the only un-arm mechanism), and braking hard enough would
   need **28 000 rev/s²** on a mid-descent retract, **280×** the declared limit. It
   shipped a third: fall back to the rest-to-rest profile — *today's exact
   behaviour*, adding no commanded magnitude the firmware could not already produce.
   **The open question**: `MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100 rev/s²` is a
   **comfort** limit, and the shipped throw profile itself commands **1902 rev/s²**
   at a 0.80 s flight and **3597 rev/s²** at `FLIGHT_TIME_MAX_S = 1.10 s` — so the
   hand has 19–36× more authority than the smooth move is allowed to use, and that
   headroom runs out at the axis's own 4178–4333 rev/s² ceiling (C-HAND-2), not at
   infinity. *(Corrected 2026-07-29 — previously "1908 / **6055** rev/s² at the band
   top … 19–60×". 6055 is the decel at the Teensy's `MAX_EVENT_VEL_MPS = 7.0`
   builder clamp, a 1.43 s flight, not at the flight band top; as written it
   asserted a routinely-commanded decel ABOVE the axis's physical ceiling. Same
   correction as § CHECK HAND-7's note and `ros_ws/docs/hand_command_continuity.md`.)*
   Whether `makeSmoothMove`
   should get a second, higher arrest limit is an **envelope decision** (it changes
   what the machine can physically do), not an implementation one. **Nothing in this
   sitting needs the answer** — the fallback *is* today's behaviour, so declining to
   decide costs nothing. It is here so you meet the question with the numbers
   attached rather than discovering it mid-debrief.
8. **Two headline criteria from the original plans are NOT reachable and have been
   replaced** (both corrected mid-run, both re-verified 2026-07-27):
   `levelling-frame-contract` Phase 4's *"commanded `rx` flat to ±0.05° across the
   whole goal"* is replaced by LVL-3's **park-plateau** criterion, and its
   *"catch error < 10 mm"* was unreachable under C-LEVEL-1 alone. C-CATCH-1 then
   removed the 0.3008° through-seat residual that caused the 16 mm, so **< 10 mm is
   now the criterion (CCATCH-2t) but it is a PREDICTION, not a measurement** —
   nothing has measured a post-C-CATCH-1 catch error yet. Treat a reading between
   10 and 16 mm as new information, not as a failure of either contract.
9. **The pre-throw tilt swing is GONE, and § Section LVL still says it remains.**
   That section was written against the C-LEVEL-1-only machine, where the swing was
   `+2.92°` at a 3.70 s lead. C-CATCH-1 removed it. § Section LVL carries a
   superseded banner; do not score `peak_above_park` against anything.
10. **The mocap `Base` body is not a plumb line, and LVL-4 has been demoted to
    REPORT because of it.** Measured 2026-07-27 on the reference bag: the parked,
    *uncorrected* Platform sits **0.087°** off `Base` while the inclinometer read
    **0.782°** off gravity at the same instant — so `Base` is itself ~0.78° off
    gravity and its local frame is a QTM body-definition artefact. Consequence: a
    correctly-levelled platform is *expected* to read ≈ 0.78° against `Base`, which
    was LVL-4's old "stop the sitting" ABORT. The gated levelling verdict is LVL-3.
    Establishing a true gravity reference for the mocap frame is unclosed and would
    need its own bench procedure.
11. **The MuJoCo plant still primes the hand at the pre-Phase-3 `9.858` rev, and a
    test pins it there.** `sim/plant/mujoco_plant.py:130` hardcodes
    `9.858 * 2π * 5.21` and `tests/sim/test_hand.py:69` asserts against the same
    literal, while the source of truth moved to **`9.9594`** in `94fe817`
    (`config/hardware_config.yaml:497`, `HAND_STROKE_TOP_REV = 9.95940313273228`).
    Nothing on this run sheet touches the sim, so it cannot affect the sitting —
    but any future session that uses the sim as the mirror for the hand path will
    reproduce the ~76.5 ms prelude Phase 3 removed on hardware and conclude the fix
    did not land. Deferred out of the 2026-07-27 close-out deliberately: it is a
    `sim/` production change plus a test change, i.e. its own logical unit.

---

## Shared preconditions (do these once per sitting)

- Jugglebot powered, ODrives up, CAN3 healthy; QTM streaming **Base +
  Platform**; `/rigid_body_poses` flowing.
- Standing rules 1–6 above apply to every check in this file.

### Build gate

**See § DEPLOYMENT MATRIX above** — it is the authority. Where a section below
says *colcon + relaunch* it means matrix row A, and running row B's two-package
build instead is always correct:

```bash
cd ~/Desktop/Jugglebot/ros_ws
colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
```

then **relaunch** `jugglebot_launch.py` — the launch runs the *installed* copy,
so a relaunch without a rebuild keeps the old code.

> ### ⚠ ONE section needs a FIRMWARE FLASH instead: § CHECK HAND-4
>
> `hand-command-continuity` Phase 4 lives in
> `ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h`, which is compiled into the
> **Platform Teensy** sketch. `colcon build` does not touch it and the Jetson never
> executes it, so **HAND-4 requires flashing `Teensy_code_platform/Teensy_code_platform.ino` to the
> Platform Teensy** (not the can-bridge, not the CatchingCone). Since 2026-07-27
> the board carries a `FW_VERSION` and reports it, so a skipped flash IS
> detectable — run-sheet row **FW-1**, and § CHECK HAND-4 row **H4.0d**. It is
> reported, not enforced: nothing refuses a command on a skew, so FW-1 is a check
> you must actually run. Every other section in this runbook is colcon + relaunch —
> and since 2026-07-29 that colcon is the **two-package** one for ALL of them
> (`--packages-select jugglebot_interfaces jugglebot`), because
> `reload_coordinator_node` imports `TossContinuous` at module scope and a
> `jugglebot`-only build now takes all three ball-op actions down with an
> `ImportError`. See § DEPLOYMENT MATRIX row B.

### Recording — ONE list, for every capture: `record:=true`

**Amended 2026-08-10 (toss-selftuning D18). There is no longer a runbook record
list. Launch with `record:=true` and you have it:**

```bash
ros2 launch jugglebot jugglebot_launch.py record:=true
```

Note the bag directory name it prints (`~/Desktop/rosbags/<stamp>`) — the
analysis commands take it as `--bag`.

**Why the hand-rolled list was retired rather than kept in sync.** Until
2026-08-10 this file carried its own `ros2 bag record` command and
`jugglebot_launch.py` carried a different one, and **neither was sufficient**:
this one had `/rosout` and `/catch/pretilt_hold` but *not* `/balls` or
`/mocap_data` — the two topics the mocap landing offset and the tracker join
both need — and the launch list had exactly the reverse. Two lists is not a
maintenance annoyance; it is a mechanism for shipping a capture that cannot
answer its own question, and it did: `~/Desktop/rosbags/2026-08-10_16-30-44`
carries none of `/rosout`, `/catch/armed`, `/catch/pretilt_hold`,
`/trajectory/commanded_position` or any action feedback, so no amount of later
analysis can recover what the catch latch or the commanded pose were doing during
that sitting. The launch list is now the **union** and it is the only list.

Two consequences worth stating:

- The old command also named `/platform_target`, which has had **no publisher**
  since the SocketCAN decommission (`can_node` is deleted). It is not in the
  union, and its absence from a new bag is correct, not a regression.
- A recorded silent topic costs nothing, and every `record:=true` bag now carries
  several by design — `/leg_lengths_topic`, `/motion/*` (dormant since the MPC
  parking) and `/catch/pretilt_hold` on an 8a build. Read a 0-message channel as
  "that publisher was not running", never as a fault.

Why each of the later additions is in the shared list rather than per-section:

- `/trajectory/status` (added 2026-07-26) — CHECK LG-4's staleness diagnostic
  measures its inter-arrival gaps, and that is only obtainable if the topic was in
  the bag **from the start**. 5 Hz; the cost is negligible.
- `/trajectory/diagnostics /trajectory/target_feedback /catch/dynamic_target
  /gravity_offset /throw_announcements` — the install census and the replay seed
  that `tools/probes/catch_reach_replay.py` needs (CATCH-2, CCATCH-2, CCATCH-3,
  ZSEAT-3), and `/trajectory/diagnostics` is also where CCATCH-5 and LVL-2 read
  `peak_leg_*`.
- `/hand_telemetry` — lets `tools/probes/hand_stroke_timeline.py --bag` work as a
  fallback. **It is not a substitute for the trace recorder** (below), which
  remains mandatory for every HAND check.
- `/catch/pretilt_hold` (added 2026-07-28) — the Tier-8b pre-tilt suppression gate.
  It is published **only** on Tier 8b, and 8b became the shipped default on
  2026-07-28, so from now on every toss capture can carry it. (**Amended
  2026-08-10**: the default went back to **`8a`**, so a default-build capture
  carries **no** `catch/pretilt_hold` messages — record the topic anyway, but read
  its absence as "8a build", not as a fault. See § SECTION TIER.) § SECTION TIER row
  `TIER-D` reads it directly; a gate left **raised** past a terminal is the failure
  it exists to catch, and that is unrecoverable from a bag that lacked the topic.
  1-2 messages per goal; the cost is nil. (`/rosout` carries the same transitions as
  `catch/pretilt_hold raised` / `released` log lines, so an older bag is not
  worthless — but it is a text fallback, not the topic.)

**AND, in its own terminal, the trace recorder** — mandatory for every capture,
because the dip lives in `hand_telemetry` at a resolution the bag path does not
reproduce, and LG-5 reads only the jsonl. **System `python3`, ROS sourced, NOT the
venv** (standing rule 5):

```bash
cd ~/Desktop/Jugglebot
python3 tests/hardware/toss_trace_recorder.py record
```

Writes `temp/logs/toss_trace_<stamp>.jsonl`; note the filename. **Confirm its 1 Hz
live line shows `hand ~100 Hz` before any goal is sent** — `hand 0 Hz` means the
Teensy telemetry stream is down and no HAND verdict is possible.

Stop and restart the recorder at the end of CAP-GATE (see § CAPTURES): LG-5
requires exactly two `loaded-flips` in one trace file, and CAP-RELAUNCH's second
`level` would add a third.

---

## Section FK — `fk-convergence-tolerance` Phase 1 (FK convergence criterion)

**Plan**: `plans/archived/fk-convergence-tolerance.md` § Phase 1
**Logbook**: `logbook/2026-07-25-fk-convergence-tolerance.md`
**What landed**: `leg_lengths_to_pose`'s bare absolute residual tolerance
(`tol=1e-10` mm) was below the achievable double-precision round-off floor in
part of the workspace, so a *fully converged* solve could be declared a failure.
`trajectory_node`'s seed path turns that into a hard **"not streaming until a
valid state"** refusal — observed as 26 consecutive ERRORs across 286 ms on
2026-07-24 09:08:55. Replaced with a mixed absolute/relative test plus a
stagnation exit, genuine-divergence raise unchanged.

**Build needs**: **colcon + relaunch** (`motion/ik_solver.py` changed).
**Firmware flash: NOT required for this section.** `controller/hardware_plant.py`
changed by comment + one named constant only; `run_mpc.py` runs it from the repo
and picks it up on next start with no build step.

**Motion expected from this section: none beyond a normal seeded hold.** This fix
must be *invisible* in the commanded stream — it removes a refusal, it does not
change any command. Fold these checks into the front of any other powered
sitting rather than booking a run for them.

### Pre-flight — confirm the freshly-built code is live

```bash
grep -q "FK_STALL_CEILING_MM" \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/motion/ik_solver.py \
  && echo INSTALLED_OK || echo INSTALLED_STALE
```

- **PASS**: prints `INSTALLED_OK`.
- **ABORT**: prints `INSTALLED_STALE` — the *installed* copy predates the fix, the
  colcon build did not take, and every check below would be testing the old
  criterion. Rebuild and relaunch before continuing.

### CHECK FK-1 — no spurious seed or guard-descent refusals

Validates: `fk-convergence-tolerance` Phase 1, the primary symptom.

Run any normal session that seeds a hold. Enter STANDBY → TRAJECTORY **at least
twice**, including once from the parked `z ≈ 170` mm hold pose where the failure
clustered (±1 encoder dead-band around the 2026-07-24 burst pose failed 96.8 %
of the time before the fix, vs 0.0 % at `z=50`/`z=110` flat — so the parked pose
is the one that exercises it).

```bash
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -c "seed FK failed" 2>/dev/null
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -c "guard descent FK failed" 2>/dev/null
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -c "non-finite target extensions" 2>/dev/null
```

- **PASS**: `0` occurrences of `seed FK failed`, `0` of `guard descent FK
  failed`, across every node log of the session.
- **ABORT**: `>= 1` of either. Baseline being replaced: **26** in 286 ms
  (`~/.ros/log/python3_198327_1784848076544.log`, 2026-07-24 09:08:55) and **1**
  on 2026-07-25 15:24:29 (`~/.ros/log/python3_31420_1784956973167.log`).
- **REPORT, do not abort**: `>= 1` of `non-finite target extensions`. That is the
  new guard firing on a NaN/±inf `pos_estimate` from the can-bridge — a real
  telemetry defect the old code masked as a generic non-convergence, not an FK
  regression. Capture the bag and open it against the can-bridge, not this plan.

### CHECK FK-2 — the offline verdict on the session's bag

Validates: `fk-convergence-tolerance` Phase 1. **This is the analysis command
that turns the capture into a verdict.**

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/fk_convergence_bag_check.py --json
```

(defaults to the newest bag under `~/Desktop/rosbags`; add `--bag <dir>` to pick
one. ~92 s per bag. JSON lands in `temp/probes/`.)

- **PASS**: banner reads `VERDICT: PASS` — the `def_rai` column is **0** for
  BOTH `/robot_state` and `/leg_setpoint_echo`, **and** `hist_rai` is `> 0` for
  at least one topic. Exit code 0.
- **ABORT**: `def_rai > 0` on either topic (exit code 1). A reconstruction still
  fails at the shipped default ⇒ the criterion did not close the failure.
- **NOT A PASS — RE-RUN ON A RICHER SESSION**: banner reads `VERDICT: VACUOUS`.
  `hist_rai == 0` means the session never visited a floor-limited pose, so a
  clean `def_rai` proves nothing. Re-run on a session that parks at `z ≈ 170` and
  moves through the catch envelope.

Reference numbers, bag `2026-07-25_15-17-48` (pre-fix data, post-fix code):

| topic | n | `def_rai` | `hist_rai` | worst accepted residual | `max_it` |
|---|---|---|---|---|---|
| `/robot_state` | 28953 | 0 | 514 (1.775 %) | 9.07e-10 mm | 5 (was 50) |
| `/leg_setpoint_echo` | 10453 | 0 | 37 (0.354 %) | 8.89e-10 mm | 5 (was 50) |

### CHECK FK-3 — no motion change (regression guard)

Validates: `fk-convergence-tolerance` Phase 1's claim that the fix is invisible
in the commanded stream. The FK-recovered pose is arithmetically identical to
`<= 4.6e-13` mm, so any *visible* difference means something else changed.

```bash
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "seeded hold at pose"
```

The line prints to **1 decimal place** (`trajectory_node.py:927`), so the
criterion is stated at that resolution — a 4.6e-13 mm difference cannot show up
in a printed 0.1 mm digit, and anything that does show up is not this fix.

- **PASS**: every seeded hold pose is the physically-expected parked pose —
  printed `z` is `170.3` (or whatever the same physical stand-still printed
  pre-change, **matching to the last printed digit**), and printed `x`, `y` are
  each within **±2.0 mm** of `0.0`. AND the `max_it` column from CHECK FK-2 is
  `<= 10`.
- **ABORT**: any printed digit of a seeded hold pose differs from the equivalent
  pre-change baseline for the same physical stand-still (i.e. a difference
  `>= 0.1 mm`), or `max_it > 10` (the criterion is not exiting where the
  13001-pose sweep says it should — worst measured is 5).

### CHECK FK-4 — MPC hot loop untouched

Validates: `fk-convergence-tolerance` Phase 1 step 4 (back-compatibility for the
one caller that passes an explicit `tol`). **Only if `run_mpc.py` is exercised
this sitting** — skip otherwise and record it as skipped.

```bash
ls -t temp/logs/mpc_*.log | head -1 | xargs grep -c "FK did not converge"
ls -t temp/logs/mpc_*.log | head -1 | xargs grep -c "fk_convergence_failure"
```

- **PASS**: `0` and `0`, and the telemetry `fk_iterations` column stays at its
  usual **2–3**, with a maximum of **5** over the run.
- **ABORT**: any `FK did not converge; using last measured pose` warning, or any
  `fk_convergence_failure` e-stop.
- **REPORT, do not abort**: `fk_iterations` reaching **10**. That is
  `max_iter` — the value cannot exceed it by construction, so a 10 means the
  budget was exhausted on that tick. See the note below.

This path already passed `tol=1e-4` mm and was never affected by the criterion
on any call that converges. One deliberate change exists and is expected to be
unobservable: on *exhaustion* (all 10 Newton steps spent), a solve whose final
step lands under `1e-4` mm now returns that pose instead of raising, so such a
tick no longer counts toward the 5-consecutive-failure e-stop. An
`fk_iterations = 10` with **no** accompanying warning is exactly that path
firing — it used to appear as a warning plus a stale-pose substitution. Report
the tick count and the pose continuity around it; do not abort.

---

## Section HAND — `hand-command-continuity` (post-throw dip + throw truncation)

**Plan**: `plans/archived/hand-command-continuity.md`

Phases 1, 2 and 4 append their own `CHECK HAND-n` bodies under this header as
they land. This first part is the **shared instrument and its pre-fix
baseline** — the numbers a post-fix capture is scored against. It exists here
because the baseline was measured offline from three 2026-07-25 sessions and
lives nowhere else; without it the Phase-5 criterion (`dip_below_x3 <= 0.10` rev,
row 4 below) has nothing to compare to.

**Nothing in this part actuates the robot.** It is read-only analysis run
*after* a capture that a later `CHECK HAND-n` produces.

### Capture requirement for every HAND check

The dip lives in `hand_telemetry`. The § Recording ONE list (`record:=true`)
**does** carry it as of 2026-08-10 — but at the launch's 100 Hz publish rate, not
the recorder's own sampling, so the two are not yet known to be equivalent (the
per-toss record's `plant_block_source: trace|bag` field exists to settle that).
Until it is settled, run the toss-trace recorder alongside the bag, in its own
terminal, with **system `python3` and the ROS env sourced — NOT the venv**
(`tests/hardware/toss_trace_recorder.py`'s own docstring states this, and
`tests/hardware/session_phase8_toss_trace.md:100` gives the same instruction; the
*probe* below is the opposite — it needs the venv):

```bash
cd ~/Desktop/Jugglebot
python3 tests/hardware/toss_trace_recorder.py record
```

(writes `temp/logs/toss_trace_<stamp>.jsonl`; note the filename. Confirm its 1 Hz
live line shows `hand ~100 Hz` before any goal is sent — `hand 0 Hz` means the
Teensy telemetry stream is down and no HAND verdict is possible.)

**The trace recorder is not optional for a HAND check** — but the *reason* has
narrowed. It used to be that the runbook's hand-rolled record command carried
neither `/hand_telemetry` nor `/throw_announcements`, so the probe's `--bag` path
had nothing to read. Since 2026-08-10 the ONE list (`record:=true`, § Recording)
carries **both**, plus `/rosout`, so a `record:=true` bag feeds
`hand_stroke_timeline.py --bag` directly. What the bag path still does not give
you is the trace recorder's own resolution — and whether that limitation is real
is now a **measurement** rather than an inherited claim: the per-toss record
carries `plant_block_source: trace|bag` precisely so the two can be compared on
the same sitting (toss-selftuning § 8). Until that comparison has been run, keep
running the trace recorder for a HAND check.

`arms` reads `?` only when the source genuinely has no `/rosout`, which is true
of the three 2026-07-25 evidence bags (recorded before that list) but not of a
`record:=true` bag. `?` never means zero; the launch log
`~/.ros/log/<stamp>/launch.log` is the fallback source.

### The analysis command — this is what turns a capture into a verdict

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/hand_stroke_timeline.py --trace temp/logs/toss_trace_<stamp>.jsonl --json
# or, from a bag:
python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/<stamp> --json
```

Self-check that the instrument itself is intact (offline, no capture needed):

```bash
python tools/probes/hand_stroke_timeline.py --gate
```

- **PASS**: **exit 0**, and **two** `GATE PASS` lines — one for the Context-table
  branch and one for `fixed-shape branch` (`clean`, `overshoot`, `short-flight`,
  `braking-prelude`, `deep-brake`, `arm-prelude+28ms`, `arm-prelude+62ms`,
  `late-trunc`, `band-floor` — the last four were added 2026-08-18/20 with the
  truncation-criterion work and are the two-sided pin on rows 1/2 below;
  `band-floor` prints the C-HAND-3 admission speed it tested against). Judge on the exit code and on
  both lines being `GATE PASS`, **not** on the row or case count: both grow
  whenever a case is added, and treating a count as the criterion has already
  produced one stale runbook.
- The same self-check now also runs in the pytest suite
  (`tests/motion/test_hand_stroke_timeline_probe.py`), so a `--gate` regression
  is caught at commit time rather than at the bench. It was not, before
  2026-08-18: the gate had been **RED on the shipped tree** since the hard-stop
  correction (11.1 → 10.8 rev) left `_GATE_EXPECT`'s `headroom_to_limit_rev` row
  on the old anchor, and nothing in the suite ran it. Running the command here is
  still mandatory — it is what proves the tree you are about to score with is the
  tree the suite passed on.
- **ABORT the analysis** (not the sitting): a `GATE FAIL` line, a missing second
  branch, or a non-zero exit. Any of those means the probe or the reference
  changed and no HAND verdict below can be trusted. `GATE UNAVAILABLE` is
  different: it means neither the recorded trace nor the committed fixture at
  `tools/probes/data/hand_stroke_timeline_gate_ref.jsonl` is present — restore
  the fixture (or regenerate it with `--emit-gate-fixture`) and re-run.

### PASS / ABORT per throw, read off the probe's rows

**Score the rows in the order below** — `peak` gates the end stop and bounds what
`pullback` can legitimately read, so a `peak` ABORT is decided before `pullback`
is looked at.

| # | row | PASS | ABORT |
|---|---|---|---|
| 1 | `trunc` | `-` (the command followed the decel ramp to `x3`) | any instant printed ⇒ the queue was cleared **while the stroke was still short of `x3`**. Record the printed `trunc` POSITION: a real clobber freezes `pos_cmd` **2–4 rev short of `x3`** (pre-fix range 6.20–7.78 rev against `x3` 9.9594). Until 2026-08-20 a reading **~0.05 rev short** was the band-floor self-trigger rather than a clobber; the collapse floor is profile-relative now and that case cannot occur, so score on presence and use the position as corroboration. Cross-read row 8: on a real truncation `stroke_end` still prints an instant — the **replacement** move reaches `x3` 208–299 ms later, and the row is annotated `<-- this is the REPLACEMENT move, not the stroke` — so a printed `stroke_end` is **not** evidence against the ABORT |
| 2 | `seeds` | `0` (printed as `-`) | `>= 1` from-rest quintic seed inside the stroke. `seeds` is reported only when `trunc` fired, so rows 1 and 2 abort together by construction |
| 3 | `peak` | **TIER-DEPENDENT since 2026-07-28 — use the § CHECK HAND-7 ladder table, not one number.** `<= 10.060` rev (`x3` 9.9594 + 0.10) is the band for the 0.38 m and 0.6 m tiers only; at and above 0.78 m the band is `<= 10.39` rev (the C-HAND-2 pessimistic bracket). The old single `<= 10.060` ABORTED on **10 of the 17** tosses of the 2026-07-27 sitting for a reason that was never a Phase-4 regression — ballistic coast growing as v² — and scoring the post-flash sitting against it would abort a working fix on the tier it exists to fix | `> ` the tier's band. **The hard abort is `> 10.60` rev** — 10.60 rev is the excursion clamp's own ceiling (`10.8 − 0.2` since 2026-08-18; `11.1 − 0.5` before — same 10.60), and `10.060 < peak <= 10.60` is a **section** abort with a specific suspect (H4.4), *not* an E-STOP. `> 10.60` rev is the HARD ABORT + E-STOP (H4.5). One number, one response |
| 4 | `dip_below_x3` | `<= 0.100` rev (`<= 3.2` mm) — the row prints `OK` | `> 0.100` rev — the row prints `OVER`. Pre-fix range was **0.339–1.748 rev = 10.7–55.3 mm**. **Qualified by row 7 after Phase 4** — see below |
| 5 | `pullback` | `>= -5.0` rev/s, **given row 3 passed** | `< -5.0` rev/s. Pre-fix range was **−17.9 to −42.4 rev/s** |
| 6 | `catch_desc` | present, within ~20 ms of `event − t_acc_catch` | absent ⇒ the catch never fired; check the Teensy serial for `Not enough time for smooth-move` |
| 7 | `first_neg_cmd` | equal to `catch_desc` (no annotation printed) | annotated `<-- NOT the catch descent (a brake?)`: **REPORT, do not abort.** Expected after Phase 4 lands (step 3 charters a braking prelude); before Phase 4 it means an unexplained downward command. Phase 4's measured reality narrows this: a braking prelude only appears once the hand's live `\|vel\|` exceeds the 6.0 rev/s dead-band, and the settle tail after a completed stroke reads `<= 0.25` rev/s, so on a clean capture this row should still read `catch_desc`. See § CHECK HAND-4 |
| 8 | `stroke_end` | an instant, followed by **either** a hold in ms (a command landed before the catch descent — see row 9) **or** `held it until the catch descent` (none did). Both are healthy: on `2026-07-27_15-39-38`, 9 of its 17 tosses print a hold (34.9–103.7 ms) and 8 print the phrase. **REPORT, never gated** — but it is the evidence behind rows 1 and 2, so record it | nothing here aborts on its own. `-` on a toss whose stroke started means the command never reached `x3` at all — row 1 will *normally* have aborted, and this row says how far it got. Not a guarantee: on a capture that reaches `x3` nowhere the scan falls back to the modelled-end + 50 ms bound, so a bare `-` here with row 1 also clean is a **hard REPORT** — re-read the window with `--preview` before scoring the toss. When `trunc` printed, the annotation warns that this instant belongs to the REPLACEMENT move, not to the stroke |
| 9 | `post_stroke_cmd` | an instant `+30…+130` ms after `stroke_end`, or `-` (no command landed before the descent). **REPORT, never gated.** The `\|cmd−meas\|` figure says *which kind* of command it was, and it agrees with row 7 on 44 of 44 post-stroke commands in the 2026-07-27 evidence base: **`≈ 0.000x` rev (measured 0.000000–0.000452, 25 tosses) = a from-rest re-seed AT the live encoder — the gated catch arm's own prelude, i.e. Phase 1 working**; **`0.06–0.17` rev (19 tosses, every one with row 7 annotated) = a Phase-4 velocity-continuous BRAKE diving below `x3`**, expected on the faster tiers and scored under rows 4 and 7, not here | nothing here aborts on its own. Route two cases: a `≈ 0.000x` re-seed on a toss where row 7 is ALSO annotated is `makeSmoothMove`'s cannot-fit fallback at the stroke top — `H4.6`'s second suspect, record the `vel_meas`; and a `\|cmd−meas\|` in neither band (roughly 0.001–0.05 rev) is unexplained — record the sample |

**Rows 1 and 2 key on the stroke END, not on a time margin — and that is a
2026-08-18 correction to a criterion that used to cry wolf.** The probe scans for
a truncation only while the commanded profile is still short of `x3`; the scan
stops the moment `pos_cmd` reaches the stroke end. Before that, the scan ran for a
fixed 50 ms past the *modelled* end — and Phase 1's arm gate withholds the catch
arm until the stroke completes and then dispatches on the next balls tick, so the
gate **working** lands a from-rest prelude just past `x3`, seeded at a live
position that has sagged 0.06–0.17 rev under it. That is a command with near-zero
commanded velocity, below `x3`: indistinguishable from a truncation to the old
predicate. On the 2026-07-27 sitting those arms landed **36.7–127.9 ms past the
modelled stroke end**, straddling the 50 ms wall: **6** tosses reported a truncation across
that sitting's six bags and **3** across its three traces, and on `2026-07-27_15-39-3x/5x`,
recorded *both* ways, the same physical tosses read **4** through the bag against
**3** through the trace. Every one was adjudicated PASS by hand. Moving the wall only relocates it: the arm's arrival is
a *scheduling* quantity with no lower bound — Phase 1's gate is designed to
dispatch on the first balls tick after the stroke completes, so an arm at +5 ms is
the gate working, and the dispatch shift alone moved +40 ms between sittings —
while the *latest* instant a truncation is still detectable is **fixed** at
~7–11 ms before the modelled end, where `pos_cmd` enters the 0.05 rev band around
`x3`. One edge is pinned, the other is a coincidence of today's tick phase, so any
wall between them buys a blind spot on late truncations the first time a tick
lands early. Keying on stroke completion has no such trade,
and an arm that lands **before** the command reached `x3` is a genuine Phase-1
gate failure that still aborts row 1. Full reasoning:
`logbook/2026-08-18-trunc-criterion-stroke-end.md`.

**What this changes for the operator, concretely.** A toss that used to print
`trunc` + `seeds=1` at the stroke top now prints `trunc = -`, `seeds = -`, and
the same event under rows 8/9 as `post_stroke_cmd`. Nothing became invisible —
what changed is which row owns it. **Do not** score `post_stroke_cmd` as row 1.

**"Collapse" is profile-relative too, since 2026-08-20 — and it used to fire on
slow throws.** The other half of row 1's predicate localises *where* the command
froze, and it did that with an absolute **10 rev/s**. That is a fixed velocity
judging a ramp whose velocity scales with the throw, so at the slow end of the
band the ramp fell under it while still short of `x3` and the probe reported a
truncation on a perfectly clean stroke. At the C-HAND-3 admission floor
(**2.440 m/s**, 0.4949 s flight) the exposure was **1.94 ms — about 1 toss in 5**;
measured end-to-end by sweeping the sampling phase, **20 of 100 phases fired**
(13 at 2.550 m/s, 6 at 2.6971, 2 at 2.800, 0 at 2.845 and above). Note the
envelope floor **moved down** with C-HAND-3, from 2.6971 to 2.440 m/s, so the
exposure was three times worse than when it was first characterised.

The threshold is now the modelled stroke's **own** commanded velocity at the
stroke-end band edge (`x3 − 0.05` rev): 8.58 rev/s at the admission floor,
13.82 at 3.93 m/s, 15.31 at the ceiling. Because the firmware's decel segment is
constant deceleration, commanded velocity falls monotonically with commanded
position, so an intact stroke short of `x3` is **never** below that floor — the
self-trigger window is empty at every speed rather than narrow (0 of 100 phases
at each speed above; 0 over 960 clean captures spanning 0.70–7.00 m/s).
Detection is untouched: the two real 2026-07-25 truncations freeze at 0.010 rev/s
against a 13.82 rev/s floor, a **1382×** margin.

**What this means at the bench.** A `trunc` on a slow toss is no longer
ambiguous — score row 1 on its presence, as written. Pinned by the `--gate`
`band-floor` case and by `tests/motion/test_hand_stroke_timeline_probe.py`. The
position cross-read in row 1 is still worth recording, but it is now
corroboration rather than the discriminator.

**Row 4 is qualified by row 7 once Phase 4 is flashed, and the qualifier is not
optional.** Row 4 measures `pos_meas` below `x3` and aborts above 0.100 rev. A
velocity-continuous BRAKE — the shape Phase 4 charters for the
at-target-but-moving case — is *also* a commanded excursion below `x3` when the
hand is moving downward, and the firmware honours it out to **3.213 rev below
`x3`** (the deepest the excursion clamp and the 0.8005 s duration cap allow, at
`v0 = 20.3` rev/s). So on a toss where row 7 prints its `<-- NOT the catch descent
(a brake?)` annotation, rows 4 and 7 score the same event in opposite directions:
row 7 says REPORT, row 4 says ABORT. **When row 7 is annotated on a toss, score
row 4 for that toss against the brake's own turning point, not against `x3`, and
treat it as REPORT** — record `dip_below_x3`, the `vel_meas` at the brake, and
which command landed. A clean capture is unaffected: the measured settle tail is
`<= 0.25` rev/s, inside the 6.0 rev/s dead-band, so no brake fires and row 7 reads
`catch_desc`. Without this qualifier an operator aborts a working fix on the first
toss that exercises the branch the phase exists to serve, and burns the sitting.

**Why row 4 and not `dip_bottom` / `dip`.** `dip` is peak-minus-bottom, so it is
non-zero on *any* capture that overshoots and settles — including a perfectly
fixed one (0.6 mm on the probe's own clean synthetic) and including the bounded
overshoot plan Phase 4 step 2 makes the *expected* behaviour. Gating on it would score
a working fix as FAILED. What separates the defect from a healthy stroke is the
*sign* of the excursion about the stroke end: pre-fix the position loop yanks the
hand **below** `x3`; a healthy stroke settles **onto** `x3` from above and never
goes under (the `--gate` synthetic post-fix shapes read 0.000–0.001 rev). Same reason
`pullback` is bounded rather than required non-negative: a healthy settle from the
coasting peak is genuinely negative — −0.31 rev/s at 0.02 rev of overshoot, −1.58
at the 10.060 rev ceiling of row 3, −10.03 at 10.60 rev.

**Row 3 governs; 10.60 rev is a CEILING, not an expectation.** The probe's
`overshoot` synthetic and the `pullback` figures above run out to 10.60 rev
because that is where Phase 4's excursion clamp caps a velocity-continuous
prelude (`10.8 − smooth_move_excursion_margin_rev 0.2`; read `11.1 − 0.5` until
the 2026-08-18 hard-stop correction, which held the ceiling at 10.60 exactly).
A clean post-fix capture
must not go near it: Phase 4 measured the hand's live `|vel|` in the +20…+70 ms
window the gated arm lands in at **≤ 0.25 rev/s**, which is inside the 6.0 rev/s
dead-band, so the prelude is rest-to-rest and the commanded excursion is
**≤ 0.0005 rev = 0.02 mm**. Row 3's `<= 10.060` rev therefore still applies
unchanged after Phase 4, and a reading between 10.060 and 10.60 is an ABORT with
a specific suspect — see § CHECK HAND-4 row H4.4.

Margins to be aware of when scoring: row 4's band has ~100× headroom on the
healthy side but only **3.4×** on the tightest pre-fix defect (ball 34, 0.339 rev).
A post-fix reading anywhere near 0.2–0.3 rev is not a clean PASS — capture it and
debrief rather than waving it through.

Also capture, without gating on it this round (the release model is still
unmeasured — `plans/active/single-ball-toss.md` Phase 5 T0): the achieved flight
per toss over **>= 5** tosses at one commanded height, and the `shift` column.

### Pre-fix baseline — 7 self-tosses, 2026-07-25 (what must go away)

The `dip_below_x3` and `pullback` columns are the **gated** ones (rows 4 and 5
above); `dip` is shown only because the plan's Context table quotes it.

| session | ball | arms | `trunc` (rev) | `peak` (rev / mm) | headroom to 10.8 | `dip` (mm / % stroke) | **`dip_below_x3` (rev / mm)** | **`pullback` (rev/s)** | `shift` (ms, **bag clock**) |
|---|---|---|---|---|---|---|---|---|---|
| 15-04-35 | 34 | 2 | 7.1245 | 10.2611 / 324.5 | 0.539 rev | 20.3 / 6.4 | **0.339 / 10.7** | −17.9 | +12.8 |
| 15-17-48 | 10 | 1 | 6.7562 | 10.2513 / 324.2 | 0.549 rev | **64.5 / 20.5** | **1.748 / 55.3** | −42.4 | +19.0 |
| 15-17-48 | 11 | 2 | 6.1965 | 10.2684 / 324.8 | 0.532 rev | 52.4 / 16.6 | **1.347 / 42.6** | −36.6 | +20.7 |
| 15-17-48 | 13 | 1 | 7.1897 | 10.2813 / 325.2 | 0.519 rev | 56.6 / 18.0 | **1.468 / 46.4** | −38.2 | +15.4 |
| 15-17-48 | 17 | 2 | 6.8525 | **10.3248 / 326.6** | **0.475 rev** | 23.0 / 7.3 | **0.361 / 11.4** | −20.0 | +20.2 |
| 15-22-50 | 2 | 1 | 7.7825 | 10.1653 / 321.5 | 0.635 rev | 40.2 / 12.8 | **1.065 / 33.7** | −29.7 | +17.2 |
| 15-22-50 | 3 | 1 | 7.7004 | 10.1743 / 321.8 | 0.626 rev | 43.0 / 13.7 | **1.146 / 36.2** | −31.3 | +21.9 |

(The headroom column was re-anchored on **2026-08-20**: it read "headroom to
11.1" and 0.775–0.935 rev until then. The `peak` measurements are unchanged — only
the anchor was wrong, and one-sidedly optimistic by 0.3 rev / 9.5 mm. See
`logbook/2026-08-18-hand-end-stop-corrected.md`.)

Worst pre-fix case: **55.3 mm below the stroke end** (ball 10) and **0.475 rev =
15.0 mm** of headroom to the end stop (ball 17), both at a mid-band
3.93 m/s throw. Every one of these seven rows must read `trunc=-`, `seeds=-` and
`dip_below_x3 <= 0.100 rev` after the fix.

**The `shift` column is the BAG-clock reading.** The same physical toss reads
**1.5–1.7 ms higher** through the jsonl trace path the verdict command above
leads with (ball 34 +14.5 vs +12.8; ball 3 +23.4 vs +21.9), because the two
sources timestamp differently. Compare like with like: a post-fix *trace* reading
of +14.3 against this table's +12.8 is a clock-path artefact, not a regression.
The honest worst case over both paths is **+23.4 ms** — that is what Phase 1's
40 ms margin is 1.7× of.

Two reload tosses in the same sessions (ball 4, ball 5, thrower `ball_butler`)
report `no-throw-stroke` with a clean `catch_desc` and every dip row `-` — that is
the expected inert shape for a reload and it must stay that way (Phase 1 step 4).
The eighth `jugglebot` toss in `15-17-48` (ball 14, `ABORTED_NO_RELEASE`) reports
`no-throw-stroke(!)` with **no** `catch_desc` at all; that is the dispatch-eaten
case, not a regression.

### CHECK HAND-1 — the catch arm no longer lands inside the throw stroke

Validates: `hand-command-continuity` **Phase 1** (arm gating). This is the check
that the operator-visible dip is gone **and** that it is gone by design rather
than by luck.

**Build needs**: **colcon + relaunch** (`catch_coordinator_node.py` and the new
`motion/trajectory/hand_stroke.py` changed). **No firmware flash** — Phase 1 is
host-side only. Phase 4 is the flash, and it has not landed.

**Motion to expect.** The hand should stroke up, **stop at the top and stay
there** until the catch descent. Two things that are NOT regressions:

- a **tiny** settle move of up to ~3 mm at ~25-80 mm/s some 100-500 ms after the
  throw, when the catch arm finally lands. `makeSmoothMove`'s "already there"
  dead-band is 1e-6 rev = **0.03 microns**, unreachable against a live encoder,
  and every non-empty prelude is floored at 50 ms — so the prelude is a short
  micro-correction, never exactly nothing. It is 20x smaller than the smallest
  pre-fix dip (10.7 mm) and does not register on the gated rows;
- the catch descent starting **later** than you remember relative to the throw.
  It is unchanged in absolute terms (the kind-1 event is an absolute wall-time
  invariant); only the moment the command is *sent* moved.

#### Pre-flight HAND-0 — confirm the freshly-built code is live

```bash
INST=~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -q "_throw_stroke_gate_ok" $INST/catch_coordinator_node.py \
  && test -f $INST/motion/trajectory/hand_stroke.py \
  && echo INSTALLED_OK || echo INSTALLED_STALE
```

- **PASS**: prints `INSTALLED_OK`.
- **ABORT**: prints `INSTALLED_STALE` — the *installed* copy predates the gate,
  so every HAND row below would re-measure the pre-fix baseline. Rebuild and
  relaunch.

#### Run

Capture per § Capture requirement (trace recorder mandatory), then throw **>= 5**
tosses at one height:

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```

(0.6 m = T 0.70 s. The default 0.78 m / T 0.80 s is the flight every pre-fix
baseline row was measured at, so run at least two at `throw_height_m: 0.78` for a
like-for-like comparison against the § Pre-fix baseline table.)

#### Verdict

Score every toss on rows 1-7 of § PASS / ABORT per throw. Rows 1, 2 and 4 are the
Phase-1 gate; then read the mechanism out of the launch log:

```bash
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep -c "hand stroke-busy window latched" "$LOG"
grep -c "hand catch arm withheld" "$LOG"
grep -c "stroke-busy window CLOSED" "$LOG"
grep    "hand catch arm withheld" "$LOG"
# Ordering matters for H1.7 — every withheld line must be followed by a dispatch:
grep -nE "hand catch arm withheld|Arming hand catch" "$LOG"
# The operator's catch-speed knob, echoed on every arm and in the CLOSED warning
# (there is no ROS param for it — it arrives on the catch/vel_scale topic):
grep -oE "scale [0-9.]+\)" "$LOG" | sort -u
```

| # | measurement | PASS | ABORT |
|---|---|---|---|
| H1.1 | `trunc`, `seeds`, `stroke_end`, `post_stroke_cmd`, `dip_below_x3`, `peak`, `pullback` per toss | rows 1-5 of § PASS / ABORT per throw, on **every** toss; rows 8-9 are REPORT — record them, they are the evidence behind rows 1-2 and row 1's ABORT cell tells you to cross-read them | any of rows 1-5 ABORTs |
| H1.2 | `window latched` count | **== number of jugglebot tosses** | `0` ⇒ the announcement never reached the gate (wrong `thrower_name`, or the announcement arrived before `catch/armed`); the dip may be absent for an unrelated reason and the PASS is luck |
| H1.3 | `arm withheld` count | **>= 1 per toss** | `0` while H1.2 passed ⇒ the arm was already late on its own; record it, and treat any H1.1 PASS as unvalidated for the gate |
| H1.4 | `stroke-busy window CLOSED` warnings | **0** | `>= 1`: the fit check refused to defer. Not a *new* hazard — the branch reproduces the pre-fix arithmetic exactly — but it does **not** mean the catch fired: the forced dispatch may itself be refused by the Teensy (see H1.6), because its fit check budgets the at-rest prelude while the hand is mid-stroke. Record `event_delay`, `event_vel` and `vel_scale` from the warning, then **check `catch/vel_scale` FIRST** (see the note below) before routing to `hand-command-continuity` Phase 1 step 3 |
| H1.5 | the withheld line's reported slack | **> 0.050 s** on every line. ⚠ **This is now a BOUNDARY, not a margin**: contract C-HAND-3 (2026-08-18) derives `FLIGHT_TIME_MIN_S` as the flight at which the window reaches exactly `arm_window_margin_s = 0.050 s`, so a goal admitted AT the floor sits exactly ON this gate. It used to sit 65 ms clear of it, at the hand-picked 0.55 s floor's 115 ms window | `<= 0.050 s` ⇒ the window is tighter in practice than the 50 ms modelled floor (115 ms before 2026-08-18); capture and route to Phase 1 step 3 before running more tosses. This row is also the guard on the ~23 ms bridge→Teensy transit that `required_arm_lead_s` deliberately does not model |
| H1.6 | `Not enough time for smooth-move` on the Teensy serial | absent — **✅ SCORED PASS 2026-08-21**: a full Platform Teensy serial capture over a multi-height session (4.44 → 5.61 m/s, i.e. up to the C-HAND-3 ceiling) contains **ZERO** occurrences. Longest prelude in that capture was `dur=0.76 s` against the 0.78964 s cap, so the cap did not truncate either. | present ⇒ the arm was refused wholesale and that toss's catch never fired. **Hard ABORT of the section.** Note the refusal leaves the throw stroke INTACT (`Teensy_code_platform.ino:642` returns before `packedMsgs.clear()` at `:648`), so this can co-occur with a clean H1.1 dip row — a clean dip is NOT evidence the catch happened |
| H1.7 | every toss that logged a `hand catch arm withheld` line also shows a later `Arming hand catch` (equivalently, probe `arms >= 1` on that toss) | **every withheld toss redeemed** | any withheld toss with no later dispatch ⇒ the deferral was never redeemed: the balls tick that should have dispatched it never arrived (track dropout, or a landing revision pushing `event_delay` under the 0.3 s floor — both bypass the gate, so no CLOSED warning appears either). Not a safety abort, but it is the one way withholding can turn into dropping — record it and route to `hand-command-continuity` Phase 1, which would then need a one-shot timer rather than a tick-driven retry |

**Before routing any H1.4 CLOSED warning to a tracker fault, read
`catch/vel_scale`.** The knob multiplies the armed event velocity, and the
window's right edge is `0.404 / v_armed`, so a LOW scale lengthens the required
lead and closes the window on its own with a perfectly healthy tracker. Swept
against the production velocities: at the 0.55-0.56 s flight, scale **0.45
closes it (−15 ms)**, 0.50 barely opens it (+18 ms), the 0.9 default gives
+116 ms; at 0.80 s and above the window stays open across the whole shipped
`[0.3, 1.5]` range. **At the DERIVED band floor (0.4949 s) 0.659 closes it**, so
the default carries 1.36× of headroom there rather than 1.78×. That corner is exactly HAND-1b below, so a CLOSED warning
there with a reduced scale is the knob, not the fix.

Also record, without gating: the achieved flight per toss, and the `shift`
column. **Do not treat the flight time as a Phase-1 measurand.** The pre-fix
0.887-1.091 s against a commanded 0.800 s is real, but by design the ball
separates at the decel ONSET (`x2`), and all seven measured truncations sit past
the commanded `x2` crossing (6.1965-7.7825 rev against `x2` = 5.9138 rev) — so
the ball had most likely already left the cup before the queue was cleared, and a
**null result here is expected and is NOT a Phase-1 failure**. The release model
is unmeasured; it is `plans/active/single-ball-toss.md` Phase 5 T0's measurand.

#### Optional HAND-1b — the short-flight corner (do this last, if HAND-1 passed)

The suppression window narrows with flight time: **395 ms** at 0.80 s but only
**115 ms** at the old band floor `FLIGHT_TIME_MIN_S = 0.55 s` (**superseded
2026-08-20 by contract C-HAND-3**: the admission floor is now the derived
`0.4949 s` = 0.300 m apex, so this corner is slightly tighter than the text
below assumes). Two tosses at

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.38}" --feedback
```

(0.38 m = T 0.557 s, still inside the band; the rejection bounds are now
**`> 1.617 m` and `< 0.300 m`** and the code is `REJECTED_THROW_ENVELOPE`,
naming the binding bound — `> 1.48 m` / `< 0.371 m` / `REJECTED_FLIGHT_TIME`
until C-HAND-3 landed 2026-08-20). Same verdict rows. **This is the corner where H1.4 is
most likely to fire**, which is the point of running it — and § Height reference
already flags `T < 0.7 s` as stroke-marginal for reasons unrelated to this plan,
so a MISSED catch here is not by itself a Phase-1 failure. Score H1.1-H1.7 only.

**Run this at the default `catch/vel_scale` (0.9).** A reduced scale closes the
window at this flight length by itself (H1.4's note), which would make the check
measure the knob rather than the gate.

### CHECK HAND-2 — a repack under a failed ack no longer clobbers a live stroke

Validates: `hand-command-continuity` **Phase 2** (repack guard). Same capture and
same tosses as HAND-1 — **no extra robot motion is required**; this is a second
reading of the HAND-1 capture.

The `HAND_TRAJ_CMD` ack fails 40-60 % of the time (the 2026-07-23 `ERR_TIMEOUT`
epidemic) and lies in both directions, so a failed ack re-opens the one-shot latch
and the arm is dispatched a second time — by design, capped at 2 per ball. Before
Phase 1 that second dispatch landed wherever the balls tick fell; during a toss
that was inside the stroke. It is now refused while the stroke is live and
deferred to the first tick after it.

**This is the half of C-HAND-1 that a capture CAN verify.** An *armed* stroke
produces no observable until its event time, so the arm itself cannot be
telemetry-confirmed the way the hand ladders were (`4e33b53`) — a Teensy-side
"armed stroke" field in `hand_telemetry` or `link_status` would fix that and is a
protocol change, out of scope here, recorded as a follow-up. But a repack that
clobbers a **live** stroke is directly visible: it re-seeds the queue from the
live encoder position, which the probe counts as a from-rest quintic `seed`.

| # | measurement | PASS | ABORT |
|---|---|---|---|
| H2.1 | probe `arms` column per toss | `1` or `2`, never `3` (`?` ⇒ the source has no `/rosout`; use the launch log) | `>= 3` ⇒ `_MAX_ARM_DISPATCHES` is not being honoured — a separate defect, route to `catch_coordinator_node` |
| H2.2 | probe `seeds` on any toss with `arms == 2` | `0` (printed `-`) — **the Phase-2 criterion**: both dispatches landed clear of the stroke, i.e. after `pos_cmd` had reached `x3`. Confirm on row 8/9: `stroke_end` prints an instant and `post_stroke_cmd` (if any) is after it | `>= 1` ⇒ a repack still clobbered a **live** stroke — the command was short of `x3` when the second dispatch landed. Pre-fix, every `arms=2` toss showed exactly 2 seeds, `0.0000 rev` from the live `pos_meas`. **A second dispatch landing after the stroke end is not this** — it is Phase 1's designed behaviour and reads as `post_stroke_cmd`; scoring it as an ABORT is the criterion defect corrected 2026-08-18 (see § PASS / ABORT's "Rows 1 and 2 key on the stroke END") |
| H2.3 | `dip_below_x3` on `arms == 2` tosses vs `arms == 1` tosses | both `<= 0.100` rev | a systematic difference between the two groups ⇒ the guard is only partly effective |
| H2.4 | a SAFE_ABORT during a toss, **if one occurs naturally** | the retract ladder still runs and the hand reaches `0.0` rev | the retract is refused or skipped. **Hard ABORT** — a kind-3 retract clobbering an armed kind-0 is the ONLY un-arm mechanism the Teensy offers. Do not provoke one deliberately this sitting |

Note for H2.2: at least one toss in the set must read `arms == 2`, or the criterion
was never exercised. With a 40-60 % ack-failure rate, 5 tosses give that with
> 99 % probability — but if all 5 read `arms == 1`, say so in the debrief rather
than recording H2.2 as a PASS.

### CHECK HAND-3 — the hand parks at the stroke top, and nothing misjudges it there

Validates: `hand-command-continuity` **Phase 3** (prime derived from stroke
geometry). **No extra robot motion is required** — every row below is read off
the HAND-1 capture plus that sitting's `launch.log`. Row H3.6 needs a SAFE_ABORT,
which is scored only *if one occurs naturally*.

**What changed, physically.** `JB_OP_HAND_CATCH_PRIME_REV` went from a
hand-maintained `9.858` to the derived stroke top `9.9594` rev, so **the hand now
parks 3.2 mm higher**. That is the whole motion change. The prize is small and
worth stating plainly so this check is not over-invested in: a kind-1 catch
trajectory begins at `x3 = 9.9594`, and `makeSmoothMove` prepends a prelude from
wherever the hand actually is to that first sample — so at 9.858 every "catch
from rest at the top" opened with a real 3.2 mm / 76.5 ms move. It now opens with
the hand's own settle error and the 50 ms prelude floor. **The prelude does not
disappear**; its commanded travel does.

**Build needs**: **colcon + relaunch**. **No firmware flash**, and the reason
matters because the generated `Teensy_code_platform/hardware_config.h` *did* change:
`JBOp::HAND_CATCH_PRIME_REV` is referenced by **no** `.ino`/`.h`/`.cpp` in any of
the three sketches — it is a dead `constexpr` that codegen delivers for
completeness. Verified by grep across `Teensy_code_platform/`, `Teensy_code_canbridge/` and
`CatchingCone_code/`. Flashing would change nothing; skipping the flash costs
nothing. (Phase 4 is the flash, and it has not landed.)

#### Pre-flight HAND-3a — confirm the freshly-built code is live

```bash
INST=~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -E "^JB_OP_HAND_CATCH_PRIME_REV|^HAND_STROKE_TOP_REV" $INST/hardware_config.py
```

- **PASS**: prints `JB_OP_HAND_CATCH_PRIME_REV = 9.9594` **and**
  `HAND_STROKE_TOP_REV = 9.95940313273228`.
- **ABORT**: prints `9.858`, or `HAND_STROKE_TOP_REV` is absent — the *installed*
  copy predates the change, so H3.1/H3.2 below would score the old prime. Rebuild
  and relaunch.

#### PASS / ABORT

The new prime is **9.9594 rev**. The near-band `_hand_dispatch_confirmed` reads
is `_HAND_NEAR_TARGET_REV = 0.5` either side, i.e. **[9.4594, 10.4594]** — that
one is a hard band read out of the shipped code, and it is what H3.1 aborts on.

**The settle envelope `[9.776, 10.145]` is a PREDICTION, not a measured band.**
It is the 2026-07-24 parked-top spread (`[9.675, 10.044]`, recorded at
`reload_coordinator_node.py:205`) translated up by 0.1014 rev, on the assumption
that the spread is command-anchored — the hand parks where it is told, so the
scatter moves with the target. That assumption is untested, and there is a
concrete reason to doubt it: the source comment calls it a *parked-top* spread,
its low edge 9.675 sits inside the pre-fix `dip_below_x3` range in § Section HAND
(0.339-1.748 rev below x3), and both edges are plausibly x3-anchored quantities
(throw-stroke ends plus the pre-fix dip) rather than prime-settle scatter — and
x3-anchored quantities do NOT translate when the prime moves. Note also that the
envelope is ±0.185 rev wide where `HAND_SETTLE_BAND_REV` is 0.10 rev and
§ Section HAND row 3 gates `peak <= 10.060` on that tighter figure.

So: **if the observed prime settles cluster near the untranslated
`[9.675, 10.044]` instead, that is evidence the 2026-07-24 spread was
x3-anchored, not a fault.** Record it and re-measure the reload ladder's probe
basis; do not abort and do not chase it as a regression.

**Which capture, and which command turns it into a verdict.** H3.1 / H3.5 / H3.7
are read off the same `temp/logs/toss_trace_*.jsonl` that § Section HAND already
records (the trace recorder is mandatory and runs under **system `python3`** with
the ROS environment sourced, *not* the venv — `tools/probes/hand_stroke_timeline.py`
is the opposite and needs the venv). H3.2 / H3.3 / H3.4 are plain greps of
`~/.ros/log/<stamp>/launch.log`. Every row on this check routes to
`plans/archived/hand-command-continuity.md` **Phase 3** on failure.

```bash
# H3.1 / H3.5 / H3.7 — prime ascents and their rest positions, from the trace
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/hand_stroke_timeline.py --trace temp/logs/toss_trace_<stamp>.jsonl --json

# H3.2 / H3.3 / H3.4 — from the launch log of the same sitting
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep 'Hand primed to' $LOG                       # H3.2
grep -c 'ABORTED_PRIME_FAILED' $LOG              # H3.3
grep -i 'smooth_move_hand' $LOG | grep -iE 'reject|out of range'   # H3.4
```

| # | measurement | PASS | ABORT |
|---|---|---|---|
| H3.1 | `pos_meas` at rest after a prime, from the trace | `9.776 <= pos <= 10.145` rev | outside **[9.4594, 10.4594]** ⇒ the parked hand is outside `_hand_dispatch_confirmed`'s near-band, so a lied ack reads as "not at target" and the ladder re-dispatches into a live ascent — the 2026-07-23 stutter. **DEBRIEF (not abort)** if inside the near-band but outside `[9.776, 10.145]`: nothing misjudges, but the settle envelope moved and the reload ladder's probe basis wants re-measuring |
| H3.2 | `grep 'Hand primed to' launch.log` | every line reads `9.959 rev` | any line reads `9.858 rev` ⇒ stale install, re-run HAND-3a |
| H3.3 | `grep -c 'ABORTED_PRIME_FAILED' launch.log` over the sitting | `0` | `>= 1` ⇒ the prime ladder exhausted. Cross-check H3.1: if the hand was physically at top each time, the near-band is the suspect |
| H3.4 | `grep -i 'smooth_move_hand' launch.log \| grep -iE 'reject\|out of range'` | `0` hits | `>= 1` ⇒ the target left the bridge's `[0, 10.8]` validation range (`[0, 11.1]` before 2026-08-18). Structurally impossible at 9.9594 (headroom **0.8406 rev = 26.6 mm** to the 10.8 rev hard stop; read 1.1406 rev / 36.1 mm before the 2026-08-18 correction); a hit means the YAML override was mis-typed |
| H3.5 | peak `pos_meas` during/just after a prime ascent | `<= 10.25` rev (1.6x the measured +0.186 rev overshoot at the old prime) | `>= 10.60` rev — still 0.2 rev short of the 10.8 hard stop (read "0.5 rev short of 11.1" before 2026-08-18) — overextension guard, but the prime is now 0.1014 rev closer to it than it was, so this is the row that watches that. **DEBRIEF (not abort) in `10.25 < peak < 10.60`**: overshoot beyond 1.6x the measured baseline with the end stop **0.20–0.55 rev** away (read 0.5–0.85 before the 2026-08-18 hard-stop correction) — record the peak and route to Phase 3 before further tosses |
| H3.6 | a SAFE_ABORT retract, **if one occurs naturally** | the hand reaches `\|pos\| <= 0.5` rev | it does not. The retract-descending qualifier moved up with the prime (`pos < 9.4594`, was `9.3580`) — *more* permissive, and the failure it guards (parked-top `\|vel\|` noise, 5.39 rev/s p99, faking a descent) still has **0.317 rev** of separation from the translated parked-top minimum. Do not provoke an abort deliberately |
| H3.7 | peak `vel_meas` during a full-stroke prime ascent | `<= 30` rev/s — see the calibration note below; the COMMANDED quintic peak is **24.63 rev/s** | `>= 40.0` rev/s ⇒ a prime can fake release evidence (`_THROW_STROKE_VEL_RPS = 40.0`). **DEBRIEF (not abort) in `30 <= peak < 40`**: that is ≥21 % over the commanded 24.63 rev/s, and overspeed on a smooth move is the re-seeded/clobbered-profile signature — record and route to `hand-command-continuity` Phase 3 |

**H3.7 calibration — score against what the firmware COMMANDS, not against the
guard's bound.** `makeSmoothMove` emits a quintic, and a quintic over Δ peaks at
`|Δ|·1.875/T` = **24.63 rev/s** at the full 9.9594 rev stroke (was 24.50 at
9.858). The figure `√(a·Δx)` = 31.56 rev/s that `_THROW_STROKE_VEL_RPS`'s comment
and `test_stroke_watch_threshold_clears_smooth_move_prelude` quote is the peak a
*bang-bang* profile would reach — a valid conservative bound for a guard to
clear, but **28 % above what the hand is actually told to do**. This row was
first written with `PASS <= 35` against the bound; scoring a capture that way
accepts a 26 % overspeed (a 31 rev/s reading) as "on model", and overspeed on a
smooth move is precisely the re-seeded or clobbered profile this plan exists to
catch. Model of record: `hand_stroke.smooth_move_peak_vel_rps` (bound:
`smooth_move_peak_vel_bound_rps`). A healthy capture should read **~24-27 rev/s**
once tracking error and the ODrive's velocity estimate noise are allowed for; a
reading *below* ~20 rev/s on a full-stroke ascent means it was not a full stroke
(check H3.1 for where the hand started).

**What a clean HAND-3 does *not* prove.** Nothing here observes the prelude the
change actually shortens — an armed kind-1's prelude produces no ROS-visible
signal, the same limitation § CHECK HAND-2 records. What HAND-3 verifies is that
moving the park position 3.2 mm broke none of the windows that read it. The
prelude claim itself is pinned offline, in
`tests/motion/test_hand_stroke.py::test_prime_at_the_stroke_top_costs_no_commanded_prelude_travel`.

---

### CHECK HAND-4 — the velocity-continuous prelude (`hand-command-continuity` Phase 4)

> ## ⚠ THIS CHECK NEEDS A **PLATFORM TEENSY FIRMWARE FLASH**, NOT A COLCON BUILD
>
> **Every other check in this runbook is `colcon build` + relaunch. This one is
> not.** Phase 4's whole deliverable is in `Teensy_code_platform/Trajectory.h`, which is
> compiled into the **Platform Teensy** sketch — the board that drives the hand
> via the can-bridge conduit. A relaunch, a rebuild, or both, change **nothing**
> about this check: `Trajectory.h` is not Python and the Jetson never executes it.
>
> Flash **`ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino`** to the **Platform
> Teensy**. Do **not** flash the can-bridge Teensy (`Teensy_code_canbridge/`) —
> it is a byte-transparent forwarder for the 0x6D0 payload and needs no change —
> and do not flash the CatchingCone.
>
> **There IS a version handshake on this board, as of 2026-07-27.** The Platform
> Teensy declares `FW_VERSION` (currently **1**) and reports it in bytes 5-6 of the
> 0x6E0 RobotState reply — row **H4.0d** below, and run-sheet **FW-1**. Read it: if
> you run this section without flashing, every row below will read exactly like the
> pre-fix baseline, and H4.0d is the only thing that will tell you why.
>
> **It reports; it does not refuse.** A skew produces an ERROR log line and a
> `link_status` field, and nothing else — no command is blocked, because the
> 0x6D0 path also carries the kind-3 retract that is the only un-arm mechanism the
> Teensy offers (`ros_ws/docs/platform_fw_version.md` § Warn, never refuse). The
> machine will let you run this whole section against a stale board.
>
> Config also changed (`config/hardware_config.yaml` gained four
> `teensy_trajectory` keys), so the regenerated `hardware_config.h` must be in the
> tree the sketch compiles against. `git pull` before opening the sketch.

**Plan**: `plans/archived/hand-command-continuity.md` § Phase 4
**Contract**: `ros_ws/docs/hand_command_continuity.md` (**C-HAND-1**, firmware half)
**What landed**: `makeSmoothMove` seeded every profile `v = a = 0` from
`current_hand_position` while `current_hand_velocity` sat declared `extern
volatile` two lines above and was **never read**. So any hand command landing
while the hand moved commanded a *velocity step*. Phases 1–2 stopped the
catch arm from being such a command; Phase 4 closes the class — a prime, a retract
ladder rung, a `SAFE_ABORT`, or a forced dispatch have the same failure shape.
The quintic is now seeded `(x0, v0, a=0) → (target, 0, 0)`, its duration solves a
corrected acceleration bound, and its overshoot is clamped against the stroke end
stops.

**Read this before scoring: the fix is a NO-OP on the clean path, by design.**
The velocity-continuous branch only engages once the hand's live `|vel|` exceeds
the **6.0 rev/s** dead-band, and the measured settle tail of a completed hand
move is **≤ 0.25 rev/s** in the +20…+70 ms window the gated arm lands in (three
2026-07-25 traces). So on a healthy toss HAND-4 should look **identical to
HAND-1** — the value of the flash is what it does to the *abnormal* paths, and to
one latent hole. Do not read "nothing changed" as "the flash did not take"; use
H4.0 for that.

**And the affordable band is narrow — that is the phase's headline finding.**
Arresting `v0` inside the stroke at the declared 100 rev/s² comfort limit costs
`0.00778·v0²` rev of travel, so continuity is affordable only up to
**~9.1 rev/s** at the stroke top (0.6406 rev of headroom to the 10.6 rev ceiling)
and **~20.9 rev/s** from the measured mid-stroke freeze at 7.7004 rev — where a
second bound binds first: an honoured prelude may not take longer than the longest
rest-to-rest move the stroke admits (0.8005 s), which caps it at **~20.3 rev/s**
anywhere. Row H4.10 reads that one. The hand
passes the release point at **~120 rev/s**, 5.7× beyond even that, so a command
landing mid-throw-stroke still takes the documented **rest-to-rest fallback** —
today's exact profile, no new commanded magnitude, and deliberately *visible* as a
`seeds` row. Rows H4.5/H4.6 are that distinction.

#### Pre-flight H4.0 — confirm the flash took (**do this first**)

```bash
# On the Jetson, with the launch NOT running: read the Platform Teensy serial.
# The smooth-move announcement line is unchanged, so use a BEHAVIOURAL probe.
#
# 1. Confirm the tree you flashed from carries the change:
cd ~/Desktop/Jugglebot && git log --oneline -1 -- ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
grep -c 'current_hand_velocity;' ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h   # expect 1
grep -n 'start_vel = current_hand_velocity' ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
grep -n 'SMOOTH_MOVE_V0_DEADBAND_RPS' ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h

# 2. Offline: compile and run the SHIPPED header and confirm it behaves. This is
#    the same check pytest runs; it needs g++ and no robot.
source ~/Desktop/PDJ_venv/venv/bin/activate
python -m pytest tests/firmware/test_hand_smooth_move_xref.py -q     # expect all pass

# 3. AFTER the flash, with the launch up: ask the BOARD what it is running.
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log; grep PLATFORM_FW_CHECK "$LOG"
```

| # | read | PASS | ABORT |
|---|---|---|---|
| H4.0a | `grep 'start_vel = current_hand_velocity'` | 1 hit | 0 hits ⇒ you are on a pre-Phase-4 tree; do not flash it, `git pull` first |
| H4.0b | `pytest tests/firmware/test_hand_smooth_move_xref.py` | all pass, **with zero skips** (the headline test compiles and runs the real `Trajectory.h`) | any failure ⇒ do **not** flash; route to `hand-command-continuity` Phase 4. **A SKIP is not a PASS**: `test_the_shipped_trajectory_h_compiles_and_agrees_with_the_mirror` skips when `g++` is absent, and it is the only thing in the repository that reads the C++ — everything else is a hand-maintained transcription. Run this on a host with `g++` (the Jetson has one) and check the summary says `passed`, not `passed, N skipped` |
| H4.0c | the sketch was flashed to the **Platform** Teensy after H4.0a passed | you flashed it, this sitting, from this tree | if in any doubt, **re-flash** — it costs a minute and it is idempotent. H4.0d is what settles it |
| **H4.0d** | `grep PLATFORM_FW_CHECK "$LOG"` — **the direct check; this is the one that decides** | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v3`. Equivalently `ros2 topic echo /link_status --once` shows `platform_fw_version: 3` (**v1 = Phase-4 only, no decel feedforward; v2 = pre-2026-08-18 end-stop correction, so `smoothMoveMaxDuration()` is still 0.8005 s and H4.10 mis-scores — treat both as a stale flash**) | `FAIL … PRE-VERSIONING` ⇒ **the board was not flashed; every row below is meaningless.** Flash and relaunch. `FAIL … v<other>` ⇒ flashed from a different tree. `UNKNOWN` ⇒ no read landed — **not** a stale flash, and usually the known benign boot-read transient: **relaunch once and re-read**, investigate CAN3 only if it repeats. **No line at all** ⇒ the `colcon build` was skipped; rebuild both packages and relaunch. This ROW SUPERSEDES the old four-link inference chain: H4.0a–c are about the tree you flash FROM, this is about the board you flashed TO. Contract: `ros_ws/docs/platform_fw_version.md`; phase: `hand-command-continuity` P6 |

#### Run

**No new robot motion.** HAND-4 is scored off the **same capture** as HAND-1
(§ CHECK HAND-1 § Run), re-run after the flash. If HAND-1 was captured before the
flash, keep that capture as the pre-flash control and take a second one after —
the pair is far more informative than either alone, and it costs one extra set of
tosses.

```bash
# the verdict command, unchanged (see § The analysis command)
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/hand_stroke_timeline.py \
    --trace temp/logs/toss_trace_<stamp>.jsonl --json

# H4.5 / H4.6 read the Teensy serial for the fallback and the refusal
```

#### PASS / ABORT

| # | row / read | PASS | ABORT |
|---|---|---|---|
| H4.1 | rows 1–5 of § PASS / ABORT per throw, on **every** toss | all PASS, exactly as HAND-1 | any row ABORTs — **except** row 4 on a toss where row 7 is annotated, which is REPORT (see § PASS / ABORT's "Row 4 is qualified by row 7"). A braking prelude is legitimately a commanded excursion below `x3` and rows 4 and 7 would otherwise score the same event in opposite directions. Otherwise: Phase 4 must not have *degraded* the clean path — that is the primary risk of this flash, because the code it changes runs ahead of every hand command |
| H4.2 | `peak` per toss, against the pre-flash capture | within **±0.05 rev** of the pre-flash reading on the same commanded height | a systematic increase ⇒ a prelude is engaging where the dead-band should have suppressed it: the hand's parked `\|vel\|` is above 6.0 rev/s at your gains/battery state. **The relevant number is the TOP-park dither**, measured once at p99 = 5.39 rev/s (2026-07-24 reload sitting) with its maximum never published — the 2026-07-25 toss traces park at the BOTTOM (≤ 2.16 rev/s) and say nothing about this risk. Record `vel_meas` at the arm instant and route to Phase 4's dead-band. **Size of the effect if it fires:** the excursion is `0.00778 * v0²` rev, so 0.280 rev = 8.9 mm just above the dead-band and 0.641 rev = 20.3 mm at the 9.07 rev/s the stroke top can absorb |
| H4.3 | `first_neg_cmd` vs `catch_desc` | equal (no annotation) on every toss | annotated `<-- NOT the catch descent (a brake?)`: **REPORT, do not abort.** It means a braking prelude fired, i.e. the hand was genuinely moving >6.0 rev/s when a command landed. Record the toss and the `vel_meas` at that instant — it is the first live evidence of the continuous branch and it belongs in the logbook |
| H4.4 | `peak`, if it lands in `10.060 < peak <= 10.60` rev | does not occur | occurs ⇒ **ABORT the section.** 10.60 rev is the excursion clamp's ceiling, so a reading in this band means a velocity-continuous prelude ran with a `v0` large enough to use most of the headroom. It is *bounded* (the clamp held) but it is not the clean path. Record `peak`, `first_neg_cmd` and `vel_meas`; route to Phase 4's dead-band and excursion margin |
| H4.5 | `peak > 10.60` rev | does not occur | **HARD ABORT, E-STOP.** The clamp is `10.8 − 0.2 = 10.60` rev *commanded* (the same 10.60; the base and margin were both corrected 2026-08-18 and the ceiling did not move); exceeding it means the clamp did not run (flash suspect — re-check H4.0), or the position loop overshot the commanded profile by more than the **0.2 rev** margin (1.08× the +0.186 rev tracking overshoot measured at the old prime; this read "0.5 rev / 2.7×" until the 2026-08-18 hard-stop correction), or the documented endpoint relaxation served a prelude from a live position *already* above 10.6 rev — which is the pre-fix measured state (10.165–10.325 rev), is legal by design, and never adds a bulge above that live reading. Check `first_neg_cmd` and the live `pos_meas` at the command instant to tell them apart. Either way the next **0.2 rev (6.3 mm)** is all that remains before metal — **there is no further guard band**. (This row read "the next 0.5 rev is the overextension guard and the 0.76 mm after that is the hard stop" until 2026-08-18; both quantities were derived from the wrong 11.1 rev anchor and over-stated the remaining margin by ~2.5×.) |
| H4.6 | `seeds` on any toss, cross-read with H4.3 **and with rows 8/9** | `seeds` = `0` (printed `-`) | `>= 1`: same ABORT as HAND-1 row 2 — the arm gate failed and a command landed while `pos_cmd` was still short of `x3` (Phase 1). Phase 4's second suspect no longer appears here: `makeSmoothMove`'s cannot-fit fallback fires from the stroke TOP, i.e. after the stroke completed, so since 2026-08-18 it reads as **`post_stroke_cmd` (row 9) with `trunc = -`**, not as a seed. **REPORT that, do not abort on it** — and note the probe's `_REPACK_STEP_REV = 0.5` rev step rule cannot see it either (the measured re-seed step at the top is ~0.156 rev), so row 9 is the only place it shows. Record which case you have, and the `vel_meas` at the command instant — above ~9 rev/s at the stroke top it is the excursion clamp, above ~20 rev/s anywhere it is the duration cap |
| H4.7 | `Not enough time for smooth-move` on the Teensy serial | absent — **✅ SCORED PASS 2026-08-21**: a full Platform Teensy serial capture over a multi-height session (4.44 → 5.61 m/s, i.e. up to the C-HAND-3 ceiling) contains **ZERO** occurrences. Longest prelude in that capture was `dur=0.76 s` against the 0.78964 s cap, so the cap did not truncate either. | present ⇒ same hard abort as H1.6. Note Phase 4 can *lengthen* a prelude (a velocity-continuous move takes longer than a rest-to-rest one over the same Δ — up to 0.24 s at the dead-band edge, 0.32 s at 8 rev/s), so if this appears **only** after the flash and H4.3 shows a brake on the same toss, the arm-fit budget needs the continuous prelude added. Route to `hand_stroke.required_arm_lead_s` |
| H4.8 | a SAFE_ABORT retract, **if one occurs naturally** | the retract still runs and the hand reaches `\|pos\| <= 0.5` rev | it does not. **Hard ABORT** — a kind-3 retract clobbering an armed kind-0 is the only un-arm mechanism the Teensy offers, and Phase 4 edits the exact condition (`makeSmoothMove` returning empty) that `Teensy_code_platform.ino:581-583` checks *before* `packedMsgs.clear()` at `:588`. The change **narrows** that branch (empty now also requires the hand to be at rest), so this should be strictly safer than before the flash — but it is the one row where a regression would be catastrophic and silent. Do **not** provoke an abort deliberately this sitting |
| H4.9 | minimum commanded/measured `pos` on any toss, and after any retract | `>= 0.0` rev — encoder zero is the excursion clamp's FLOOR and the host's own declared floor for this axis | `< 0.0` rev ⇒ **hard abort.** The bottom hard stop is at −0.1 rev (the axis homes downward into it) and the floor carries no margin for the position loop's +0.186 rev undershoot, so any commanded value below zero is planned travel onto the stop. Route to Phase 4's `SMOOTH_MOVE_POS_FLOOR_REV` |
| H4.10 | duration of any commanded hand move (last sample − first, from the trace) | `<= 0.78964` s (was `0.8005` s pre-2026-08-18; **a board still emitting up to `0.8005` s is an UNFLASHED board** — cross-read FW-1) | `>` that ⇒ the firmware's duration cap did not run (flash suspect). The cap is what keeps `catch_coordinator._PRIME_INFLIGHT_S = 1.2` s covering every profile the Teensy can emit; above it a re-prime tick can land inside a live ascent, which is the 2026-07-23 stutter |

**What HAND-4 cannot see, stated plainly.** Every row above is scored on the
*clean* path, which the fix deliberately leaves unchanged, plus two rows (H4.3,
H4.6) that only fire if the abnormal path happens to be exercised. The
velocity-continuous branch itself — the actual deliverable — has **no** row that
provokes it, because provoking it means dispatching a hand command while the hand
is moving at 6–9 rev/s, which is either the defect Phases 1–2 removed or a
deliberate abuse of the un-arm path. It is validated offline instead, against the
**compiled shipped header**:
`tests/firmware/test_hand_smooth_move_xref.py::test_the_shipped_trajectory_h_compiles_and_agrees_with_the_mirror`
builds `Trajectory.h` with g++ and checks the emitted profile, the branch
decision, the duration and the excursion interval on fifteen
`(start, target, v0)` cases including the measured 119.6 rev/s. That test, not
this section, is the evidence the profile is right; this section is the evidence
the flash did not break anything the robot does every day.

**One decision the operator owns, and it is why the fallback exists.** When the
overshoot cannot fit inside the stroke, `makeSmoothMove` reverts to the
rest-to-rest profile rather than braking harder, because the acceleration needed
to arrest sooner is not bounded by anything the firmware declares — a SAFE_ABORT
retract to 0.0 rev dispatched mid-descent at the measured −60 rev/s would need
**28 000 rev/s²**, 280× the declared 100 rev/s² limit, to fit its overshoot into
the 0.1 rev of travel above the homing stop. The declared limit is a *comfort*
limit: the shipped throw profile itself commands **1902 rev/s²** at a 0.80 s
flight and **3597 rev/s²** at the shipped `FLIGHT_TIME_MAX_S = 1.10 s`.
*(Corrected 2026-07-28. This row previously read "6055 rev/s² at the band
top"; 6055 is the decel at the Teensy's `MAX_EVENT_VEL_MPS = 7.0` clamp — a
1.43 s flight — not at the band top the rest of this file means. Re-derived
from the shipped header: `|throwD| = v²/(t_acc_coeff · INERTIA_RATIO)` =
123.55·v² rev/s².)* **Whether to give `makeSmoothMove` a second,
higher arrest limit is an envelope decision, not an implementation one** — it
changes what the machine can physically do at the bench. Nothing in this sitting
requires the answer; the fallback is today's behaviour, so declining to decide
costs nothing.

---

## CHECK HAND-7 — the post-release deceleration (`hand-command-continuity` Phase 7)

**Validates:** contract **C-HAND-2**, `ros_ws/docs/hand_decel_feedforward.md`.
**Plan:** `plans/archived/hand-command-continuity.md` § Phase 7.
**Deployment:** § DEPLOYMENT MATRIX **row C** — a **Platform Teensy flash**
(this section landed at `FW_VERSION` 1 → **2**; the board and the tree are on
**3** since the 2026-08-18 end-stop correction) — *plus* row A's `colcon build`
for the regenerated `hardware_config.py`. **Run FW-1 first. On a `v1` board every
row below is meaningless**, and it is meaningless in the most dangerous direction: a v1 board
is the board that touched the end stop.

### What this section is about, in one paragraph

On 2026-07-27 the hand made **light physical contact with its mechanical end
stop** on the five off-run-sheet ~1.2 m throws (`peak` 10.860–11.062 rev against
a declared 11.1 rev guard). Nothing commanded was at fault: `pos_cmd` never left
`x3 = 9.9594` rev. The commanded profile allocates the decel **4.046 rev of
travel at every speed** (velocity-independent — see C-HAND-2), ending exactly on
the top of the usable stroke, so it budgets the *ideal* stopping distance with
**zero** allowance for tracking error. The braking torque feedforward was sized
from a hand-mass-on-a-spool model that omits the motor's rotor, delivering ~70 %
of the torque the commanded deceleration physically needs; the rest had to come
from a position/velocity loop far too slow to supply it inside a 47–93 ms ramp.
Phase 7 corrects the feedforward. **It changes no commanded position and no
commanded velocity** — only the torque stream of the decel segment.

### What you should feel and hear

**No end-stop contact at any tier.** No click, no thud, no metallic tap at the
top of the throw stroke. On 2026-07-27 that sound was the finding; if you hear it
again at any rung, **stop climbing the ladder** and record the tier.

The stroke itself should look and sound *unchanged* — same ascent, same release,
same timing. What changes is only how firmly it stops at the top. A stroke that
sounds harsher on the way UP means the correction reached the accel segment,
which it must not: that is an ABORT, not a tuning observation.

> ### ✅ UPDATE 2026-08-20 — the whole ladder is inside the envelope again
>
> On **2026-08-18** this box said R3/R4/R5 were REFUSED by contract **C-HAND-3**
> (`ros_ws/docs/hand_throw_envelope.md`), because the envelope's coast model was
> the 2026-07-27 pre-fix ladder. **That is superseded.** A purpose-built session
> on 2026-08-20 (bag `2026-08-20_21-51-39`, six throws, all caught) measured
> coast on the flashed plant, and it is 3–5× smaller than the pre-fix model
> extrapolated:
>
> | rung | commanded | release | modelled peak | verdict |
> |---|---|---|---|---|
> | **R0** | 0.55 s | 2.709 m/s | 10.104 rev | admitted |
> | **R1** | 0.60 m | 3.440 m/s | 10.119 rev | admitted |
> | **R2** | 0.78 m | 3.920 m/s | 10.136 rev | admitted |
> | **R3** | 1.00 m | 4.436 m/s | 10.185 rev | admitted (n = 14 measured here) |
> | **R4** | ~1.20 m | 4.858 m/s | 10.230 rev | admitted |
> | **R5** | 1.10 s | 5.399 m/s | 10.294 rev | admitted |
>
> The derived band is now **`[0.4949, 1.1485]` s** = apex **0.300–1.617 m**, and
> the binding bound is **`DECEL_FF_HEADROOM`** (the decel feedforward may draw at
> most 85 % of `hand_curr_limit_a` — this section's own H7.5 requirement, now a
> config key), *not* the end stop. `END_STOP` does not bind until 7.468 m/s.
>
> **The clamp was the mechanism, and there is a within-session A/B for it.** Bag
> `2026-08-18_18-42-19` changes the braking clamp mid-session: `iq_meas` never
> passes −8.87 A through t = 0–100 s, then reaches −17.4 A after. Its first
> throw (t = 84.9 s) coasted **+0.763 rev**; every other throw in the same bag at
> the identical commanded 4.436 m/s coasts **0.18–0.23**. So H7.0c's clamp
> question is now answered in the strongest possible way: the clamp was live, it
> *was* binding the decel ramp, and removing it is what fixed the overshoot.
>
> **What this means for the ladder below.** The pre-fix `peak` bands in the rung
> table are still the right PASS/ABORT gates — they were written against the
> 10.60 rev abort line, which has not moved. But the ladder's purpose has
> changed: it is no longer proving the feedforward correction works (the A/B
> above did that), it is routine validation. Climb it if you are re-validating
> after a firmware or drive change; otherwise the coast ladder in
> `hand_throw_envelope.measured_coast_rev` is the live record, and extending it
> above 4.436 m/s is the one thing that would shrink C-HAND-3's 1.27×
> extrapolation.

### THE LADDER — climb it in order, and do not skip a rung

Each rung is scored with the **same** verdict command as § CHECK HAND-1 (the
timeline probe), plus the new authority probe:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
TR=$(ls -t temp/logs/toss_trace_*.jsonl | head -1)

# the per-throw rows (trunc / seeds / peak / dip_below_x3 / pullback / catch_desc)
python tools/probes/hand_stroke_timeline.py --trace "$TR" --json

# HAND-7's own instrument: per-tier peak, over_x3, the implied tracking fraction
# eta, decel-phase current, and the authority table.  This is the SAME command
# that produced the pre-fix numbers in the table below, so the comparison is
# like-for-like.
python tools/probes/hand_decel_authority.py --trace "$TR" --json
# or, from the bag the recording command actually produces:
python tools/probes/hand_decel_authority.py --bag ~/Desktop/rosbags/<stamp> --json
```

| rung | commanded height | tosses | `peak` PASS | DEBRIEF | ABORT | pre-fix measured |
|---|---|---|---|---|---|---|
| **R0** | `flight_time_s = 0.55` (h ≈ 0.37 m) | 3 | `<= 10.060` rev **and `dip_below_x3 <= 0.100`** | dip 0.100–0.20 | dip `> 0.20`, or `peak > 10.20` | *(not on the 2026-07-27 ladder; nearest tier 2.742 m/s → 10.0295–10.0371)* |
| **R1** | 0.60 m | 5 | `<= 10.060` rev | 10.060–10.20 | `> 10.20` | 10.0056–10.0407 |
| **R2** | 0.78 m | 5 | `<= 10.20` rev | 10.20–10.33 | `> 10.33` | 10.2851–10.3258 |
| **R3** | 1.00 m | 3 | `<= 10.39` rev | 10.39–10.60 | `> 10.60` **+ E-STOP** | *(never flown)* |
| **R4** | ~1.20 m | 3 | `<= 10.39` rev | 10.39–10.60 | `> 10.60` **+ E-STOP** | **10.8601–11.0621** ← the tier that touched |
| **R5** | the band ceiling, `flight_time_s = 1.10` (h ≈ 1.48 m) | 2 | `<= 10.39` rev | 10.39–10.60 | `> 10.60` **+ E-STOP** | *(never flown)* |

**R0 is new, and it is the rung that tests the OTHER failure direction.** Every
other rung asks "did the overshoot shrink?". R0 asks "did the correction
*over*-brake?", and it is first on the ladder because over-braking bites at the
BOTTOM of the band, not the top. The reason is gravity: on an upward decel it
brakes in the same direction as the feedforward, worth `τ_grav/(2π·a_cmd)` of
effective inertia — +1.46e-6 kg·m² at `FLIGHT_TIME_MIN_S` against a declared
9.5e-6, but only +0.37e-6 at the ceiling. So the open-loop braking exceeds the
commanded profile below `a_cmd ≈ 1900 rev/s²` and nowhere above it. The loop is
expected to absorb it (predicted dip 0.013–0.025 rev against the 0.100 gate — it
is also where the decel ramp is longest and the loop most effective), but that
prediction has never been measured, and **routine reload/catch work throws at
this height**, below every rung the ladder previously had. Its gate is
`dip_below_x3`, not `peak`. See C-HAND-2 § *The one-sided-safety clause, stated
honestly*.

**R5 is gated, not optional-at-your-discretion.** Run it **only** if R0–R4 all
passed **and** row **H7.3** below (the flatness row) passed. R5 is the first time
this machine has ever been asked for a legal in-band toss at its own configured
ceiling, and the pre-fix extrapolation said such a toss would exceed the top of
the 355 mm stroke. *(The 2026-08-18 draft of C-HAND-3 refused R5; the
2026-08-20 measurement re-admitted it — see the box above. Its modelled peak is
10.294 rev, 16 mm clear of metal, against the pre-fix model's 12.17.)*

**At R5, read `v_pk` before you conclude anything from a DEBRIEF-band peak.** The
overshoot goes as `v²`, so a release-speed overshoot moves the peak hard: the
bracket's 0.215 rev of margin is fully consumed at ε = +2.6 %, and the ~1.2 m
tier measured ε = +1.8 %, −7.1 %, +1.9 %, −0.3 %, −4.0 % on five throws. A peak
of 10.4–10.6 rev at R5 with `v_pk` ~2 % high is **release-speed scatter on a
working fix**, and the response is to record it — not to touch the declared
inertia, which would be the response to a peak that is high at the *commanded*
speed. The two have opposite fixes, so distinguish them.

**Where the bands come from — so you can judge them, not just apply them.**
R1/R2's bands are "at least as good as the pre-fix measurement at the same
height, with a margin": the corrected feedforward commands strictly more braking
torque, so the peak cannot get *worse*, and a rung that fails to improve is
telling you something real. R3–R5's `10.39` is the **pessimistic bracket** of
C-HAND-2 — what the peak would be if the feedforward were the *only* braking and
the closed loop contributed nothing, at the least favourable of the two
reflected-inertia identifications. It is velocity-independent, which is why one
number covers three rungs. `10.60` is unchanged: the excursion clamp's own
ceiling (`10.8 − 0.2` since 2026-08-18; `11.1 − 0.5` before — same 10.60), and the same hard-abort line § CHECK HAND-4 uses.

### PASS / ABORT — the rows unique to this section

| # | row | how | PASS | ABORT |
|---|---|---|---|---|
| **H7.0** | the board is on v3 | `grep PLATFORM_FW_CHECK "$LOG"` | `OK — … v3` | anything else — see FW-1. **A `v1` board scores this whole section as a re-run of the sitting that touched the stop**; a `v2` board has the feedforward but predates the end-stop correction, so H4.10 mis-scores it |
| **H7.0a** | **desk, before the flash** — the firmware xref actually ran | `pytest tests/firmware/test_hand_throw_decel_xref.py -q` on a host **with `g++`** | `passed`, **ZERO skips** | any failure, **or `N skipped`**. A SKIP means `g++` was absent, and this file is the only thing in the repository that reads the C++ you are about to flash — everything else is a hand-maintained transcription. **A SKIP is not a PASS. Do not flash on a skip.** Same lesson as H4.0b, which is about the sibling xref |
| **H7.0b** | **desk** — the verdict instrument itself is sound | `python tools/probes/hand_decel_authority.py --self-check` | `SELF-CHECK: PASS`, exit 0 — it scores a synthetic PRE-fix capture FLAG and a synthetic POST-fix capture ACCEPT | any `BAD` line. This probe produces H7.2 / H7.3 / H7.5; an instrument validated on only the broken shape scores a **working fix as a failure** and burns the sitting. Also INST-6 |
| **H7.0c** | **before the flash, on the live drive** — the negative torque clamp | `odrivetool` on the hand axis: read `axis0.config.torque_soft_min` | anything `<= -0.20` N·m (comfortably below the 38.9 A = 0.215 N·m the ceiling rung commands) | **`-0.0551` N·m** (= exactly −10.00 A) ⇒ **STOP, do not flash.** That is what `config/ODrive config Files/odrive_pro_hand_config.json` declares, and it is asymmetric against a `torque_soft_max` of +0.5 N·m. If it is live it truncates the decel feedforward — **legacy and corrected alike** — above ~0.49 m, and the whole ladder would read pre-fix numbers that the failure table below would misattribute to physics. **ANSWERED 2026-08-18: the clamp WAS live.** `torque_soft_min/max` are now a symmetric **±0.7 N·m** and `save_configuration()` has been run, so this row is a REGRESSION CHECK, not an open question — a reading of `−0.0551` now means the drive config drifted back. Original text: *record the value either way; it takes 30 s and it is currently an open question* (C-HAND-2 § *The negative torque clamp*) |
| **H7.1** | no end-stop contact | **your ears and your hand**, every toss | silence at the top of the stroke | any click / thud / tap ⇒ **STOP the ladder**, record the rung, do not climb |
| **H7.2** | `peak` per rung | `hand_decel_authority.py`, `peak` column | the rung's band above | the rung's band above. **`peak` is the END-STOP column and is deliberately the largest excursion of ANY cause** — on a healthy capture that is routinely the gated catch arm's own prelude, not the throw, and a `*` beside the value says so. Score the END STOP on `peak`; score the DECELERATION on `cst_pk` / `over_x3`, which are bounded to the throw's own coast (2026-08-23). |
| **H7.3** | **flatness — the falsifiable prediction** | `hand_decel_authority.py` prints it directly: the `flatness (H7.3)` line, **spread of the per-TIER MEAN `over_x3`** (not per-toss). Since 2026-08-23 `over_x3` is measured on the throw's own coast, so this statistic finally compares one physical event across tiers — before that, tiers whose coast was smaller than the arm's overshoot contributed the ARM | spread across **R1–R4** — R0 is excluded, it is the over-brake rung and may read at or below zero — **`<= 0.35` rev**. **A spread inside the gate means nothing if the overs are uniformly NEGATIVE** (2026-08-23: spread 0.2151 rev, an ACCEPT, on a ladder whose every rung under-shot `x3`). Flatness answers *did the overshoot stop growing with speed*; it cannot answer *is there an overshoot at all*. Read the sign first, and if the overs are negative the verdict belongs to H7.4, not here | `> 0.35` rev ⇒ the overshoot still grows with speed, so the feedforward did not become the dominant braking term and extrapolating to the ceiling is unjustified. **Do not run R5.** Pre-fix this spread was **0.9575 rev** (0.0629 → 1.0204). **Where 0.35 comes from** (it is a prediction with margin, not a round number): the pessimistic bracket is velocity-independent, but the *achieved* overshoot is `loop attenuation × open-loop bracket`, and the attenuation is strongly speed-dependent — measured 4.3 / 3.7 / 20.2 / 59.4 % of the 1.7185 rev pre-fix open-loop bracket at the four tiers. Applying those same ratios to the 0.4259 rev post-fix bracket predicts per-tier `over_x3` of 0.018 / 0.016 / 0.086 / 0.253, i.e. a **spread of 0.237 rev** for a fix behaving exactly as modelled. 0.35 leaves 1.5× headroom over that prediction while still failing the pre-fix shape by 2.7×. *(Raised 2026-07-29 from 0.25, which sat at 95 % of the model's own prediction — a coin flip that would have blocked R5 on a working fix.)* |
| **H7.4** | `dip_below_x3` **AND `coast_below_x3`** | timeline probe, rows 4 and 5 — **read both** | `<= 0.100` rev on BOTH, unchanged | `> 0.100` rev ⇒ **the correction over-braked.** This is the one failure mode C-HAND-2's "declared inertia ≤ measured" clause exists to prevent, so it is a *contract* violation, not a tuning miss: lower `teensy_trajectory.throw_decel_reflected_inertia_kgm2`, do not raise it. Record the value and the tier. **The gated row alone is not sufficient (2026-08-23):** it searches for the bottom only after the window's maximum, and that maximum can be the arm prelude's climb ~200 ms after the throw — on the first ladder flown on the unclamped drive it read `0.000` on **15 of 15** throws whose own coasts finished **0.119-0.481 rev** under `x3`. `coast_below_x3` asks the same question over the coast only and the printer marks the disagreement `<<< BLIND SPOT`. A BLIND SPOT marker is an H7.4 FAIL. |
| **H7.5** | decel-phase current | `hand_decel_authority.py`, `iq_ramp` / `iq_ramp_brake` / `fr` | `< 45` A on every toss | `>= 45` A ⇒ within 10 % of `hand_curr_limit_a = 50`. The feedforward is designed to peak at **31.6 A** at ~1.2 m and **38.9 A** at the band ceiling; a reading above 45 means the loop is adding much more than modelled. **Do not run R5.** Raising the current limit is an operator decision and is NOT the response. **Read this row as a one-sided upper bound and nothing more** — see the box below |
| **H7.6** | terminal Vel_FF and latched torque (REPORT) | `hand_decel_authority.py`, `vff_hold_max` / `tor_hold_max` / `settle_offset` | record, do not gate | Known pre-existing effect: `buildSegment` stops one 500 Hz sample short of `t3`, so the last frame carries a residual velocity feedforward (up to 7.19 rev/s at the band ceiling) **and the FULL decel torque**, both latched by the drive until the next hand command. Pre-fix `vff_hold_max` by tier: **2.02 / 0.82 / 1.41 / 1.14 rev/s** at 2.742 / 3.440 / 3.969 / 4.858 m/s (it does *not* rise with speed). **`tor_hold_max` is the one number this phase moves at the latch — it grows 1.289×**, pre-fix 0.040 / 0.070 / 0.090 / 0.140 N·m by the same tiers. Settled `pos_meas − pos_cmd`, **per tier because one span hides the point**: **+0.0669…+0.0671** (2.742) / **+0.0429…+0.0663** (3.440) / **+0.0190…+0.0412** (3.969) / **−0.0043…+0.0069** (4.858). It *shrinks* with speed and is ~0 at the tier this section exists for, so do not score a post-flash 4.858 reading against the higher tiers' band. Not caused by the Vel_FF residual (median Vel_FF through the same window is 0.00–0.02 rev/s). See `tests/firmware/test_hand_throw_decel_xref.py::test_the_throw_ends_with_a_residual_velocity_feedforward` |
| **H7.7** | achieved flight time (REPORT) | `/throw_announcements` vs the observed flight | record per rung | The accel feedforward is **deliberately untouched**, so the achieved release velocity should NOT move. A systematic change in achieved flight at a fixed commanded height means the correction reached the ascent — cross-check `tor_ff_cmd` during the ascent in the trace (it must still read the pre-fix value for that speed) |

> ### ⚠️ Judge this ladder on the KINEMATICS, not on the current
>
> The two channels this instrument reports are **not** equally trustworthy, and
> the difference decides which rows can gate.
>
> `iq_meas` is a stale field inside an already-aliased stream: `hand_telemetry`
> is a ~100 Hz snapshot of a 500 Hz stream, and `iq_meas` has a **median
> repeat-run of 4 samples**. The probe now prints `fr`, the count of genuinely
> FRESH `iq` samples inside each decel window — on the 2026-07-27 capture that is
> **0 or 1 on 8 of 17 tosses**, and the decel ramp is only 52.7–93.3 ms. So every
> current number is a **lower bound**, H7.5 can only ever catch a gross
> saturation, and *it cannot tell you the feedforward landed*.
>
> The kinematic channel (`peak`, `over_x3`, `eta`) is sampled at the same rate but
> is an *integral* of the motion, so it is robust. **H7.2 and H7.3 are the rows
> that actually decide whether the fix worked**, and H7.3 is also the
> discriminator for the H7.0c clamp question: under a live `torque_soft_min` the
> overshoot stays speed-dependent and the flatness row fails.
>
> *The decel window itself was wrong until 2026-07-29* — it was anchored on the
> announcement's `throw_time`, which lands **120.5–166.9 ms** before the commanded
> stroke end while `t_dec` is only 52.7–93.3 ms, so it had **zero overlap** with
> the real deceleration on **13 of 17** tosses and reported the ASCENT current.
> Pre-fix reference numbers below are from the **corrected** window; do not
> compare them against anything printed by an older copy of the probe.
>
> Pre-fix decel-window braking current (`iq_ramp_brake`), by tier:
> **3.6–3.9 A** (2.742) / **7.2–9.0 A** (3.440) / **4.8–7.5 A** (3.969) /
> **6.9–9.9 A** (4.858). Design values post-fix are 9.7 A at the band floor,
> 31.6 A at ~1.2 m, 38.9 A at the ceiling — so if the telemetry *were* trustworthy
> you would expect a large rise. **It is not, so a flat reading is not evidence
> of anything.** Report it; do not conclude from it.

### If a rung fails

| symptom | most likely | do |
|---|---|---|
| `peak` no better than pre-fix at R2 | the flash did not take, or took from the wrong tree | re-run **H7.0**. **Do NOT use `iq_ramp` as the corroborating evidence** — it is aliased to 0–1 fresh samples per ramp and reads flat either way (see the box above). The evidence is the FW version and `over_x3`; do not re-tune |
| every rung reads its pre-fix peak, H7.0 says the board is current, and the flatness row also fails | the **negative torque clamp is live** — `torque_soft_min = −0.0551` N·m truncates the feedforward above ~0.49 m, so the flash landed but the drive refuses the command | **STOP.** Run **H7.0c** (it should have run before the flash). This is a drive-config finding, not a physics one, and the row below would misattribute it |
| `peak` improved but not to band; `over_x3` still grows with speed (H7.3 fails) | the declared inertia is still an under-estimate — the true reflected inertia is above 1.05e-5 kg·m² | **STOP.** Rule out the clamp row above first. Then this is a measurement to bring back, not a bench tweak: re-run `hand_decel_authority.py`, record the per-tier `eta`, and hand it to the plan. Raising `throw_decel_reflected_inertia_kgm2` past the measured value would break C-HAND-2's safety clause |
| `dip_below_x3 > 0.100` at **R0 only**, peaks fine everywhere | the gravity term — expected direction, unmeasured magnitude | **STOP at R0, do not climb.** This is the one failure the R0 rung exists to catch: open-loop, feedforward + gravity exceeds the commanded decel below `a_cmd ≈ 1900 rev/s²`, and the loop was predicted to absorb it to 0.013–0.025 rev. Record the value and the tier. The fix direction is **lower** `throw_decel_reflected_inertia_kgm2` — but note no value both fixes this and clears 10.60 rev at the ceiling (8.69e-6 predicts a 10.80 rev peak), so it is a plan decision, not a bench tweak |
| `dip_below_x3 > 0.100` (H7.4 fails) | the declared inertia is an **over**-estimate | **STOP.** Lower it. This is the contract's one-sided-safety clause failing, and it is the failure this section most wants to hear about |
| `iq_ramp >= 45` A (H7.5 fails) | the loop is adding far more than modelled, or the reflected inertia is much higher than identified | **STOP, do not climb.** Record and hand back |

### What this section deliberately does NOT do

* **It does not cap the throw height.** That was the obvious remedy and the
  operator explicitly rejected it (decision (b), 2026-07-28).
* **It does not steepen the commanded ramp.** At the band ceiling the profile's
  own commanded deceleration already needs **83–86 %** of everything the axis can
  produce at `hand_curr_limit_a = 50 A`, so the steepest commandable ramp buys
  ~15 % of the overshoot. Measured, not assumed — the authority table is printed
  by `hand_decel_authority.py`.
* **It does not touch the ascent, the catch, or `makeSmoothMove`.** Rows H7.7 and
  H4.* are the guards on that.
* ~~**It does not resolve where the physical stop actually is.**~~ **RESOLVED
  2026-08-18 — the hard stop is 10.8 rev, metal contact, operator-measured on the
  sensorised hand** (the hand changed during ball-sensor integration and the limit
  was never updated). All three candidate anchors (11.124 / 11.224 / 11.4 rev) were
  too high, and the shipped **11.1 rev guard sat 0.3 rev PAST metal** — it was never
  protective. The operator's contact at a measured 11.06 rev was therefore 0.26 rev
  *past* the stop, not near it. HAND-7's bands are all at or below 10.60 rev, so the
  ladder was, and remains, safe. See `logbook/2026-08-18-hand-end-stop-corrected.md`.

> **⚠ THE R0–R5 LADDER WAS DECLINED BY THE OWNER, 2026-08-18.** catch-robustness
> Phase 0 closed on the clamp fix alone: `torque_soft_min/max` are now ±0.7 N·m and
> the post-throw dip is gone by eye, and the owner judged further bench time not
> worth it. So this section's **quantitative gate was never measured on the restored
> drive**, § CAP-DECEL is **not** the next capture, and C-HAND-2's
> `J >= 1.0126e-5 kg·m²` — measured *through* the clamp — is a watch-item rather
> than a scheduled re-derivation. The rungs stay here, unrun, for whoever wants
> them: re-derive if end-stop `peak` or `dip_below_x3` ever regresses.


---

## Section FW — `hand-command-continuity` Phase 6 (Platform Teensy `FW_VERSION`)

**Plan**: `plans/archived/hand-command-continuity.md` § Phase 6
**Contract**: `ros_ws/docs/platform_fw_version.md` (**C-PLATFW-1**)
**Run-sheet rows**: **INST-4**, **INST-5** (at the desk), **FW-1** and **FW-2**
(stage 3), **H4.0d** (§ CHECK HAND-4 pre-flight — the same check, cited where it
decides). FW-2 is the companion deployment check on the *host* half: this phase
added two fields to `RobotState.msg`, so a `jugglebot`-only rebuild now silently
kills `/robot_state`; `INTERFACES_STALE` names it at construction.

**No robot motion. No extra capture. Nothing to score after the sitting.** This
section exists because a *deployment* became observable, not because a behaviour
changed.

### What changed and why it matters

The Platform Teensy carried no version of any kind, so an un-flashed board was
indistinguishable from a flashed one from the Jetson — the only deployment in this
stack that failed silently (row A throws at a grep; row B kills `trajectory_node`
~200 ms after launch). It now declares `FW_VERSION` and reports it in **bytes 5-6
of the 0x6E0 RobotState reply it already sends** — the same frame that carries
`is_homed` / `levelling_complete` / `pose_offset`.

The property that makes this work is not economy, it is what a *stale* board does:
every firmware built before 2026-07-27 zero-filled those bytes unconditionally, at
the same dlc 8, so **a pre-versioning board answers — with 0 — rather than going
silent**. A brand-new query frame would have made the un-flashed signature an
*absence*, indistinguishable from a CAN3 hiccup or an unpowered board, and would
have needed a **can-bridge flash** to detect a stale **Platform** flash.

Consequences worth knowing at the bench:

- **No new CAN frame, no new arbitration id, no extra CAN3 duty cycle** — nothing
  was added to the bus the 0x6D0 hand conduit shares.
- **No can-bridge change and no can-bridge flash.** Do not flash the can-bridge
  for this.
- **The check is one-shot per RobotState read** (boot, UDP reconnect, CAN3
  recovery), not polled. `link_status` carries the cached value continuously.

### Reading it

| verdict | `link_status/platform_fw_version` | launch log | what it means |
|---|---|---|---|
| **OK** | `2` | `PLATFORM_FW_CHECK: OK` | flashed, current — continue |
| **FAIL, stale release** | `1` | `PLATFORM_FW_CHECK: FAIL … v1` | flashed, but only up to Phase 4. **§ CHECK HAND-7 is meaningless** — re-flash |
| **FAIL, pre-versioning** | `0 (PRE-VERSIONING)` | `PLATFORM_FW_CHECK: FAIL … PRE-VERSIONING` | the board answered and **has not been flashed** — flash, relaunch, re-check |
| **FAIL, other release** | the number | `PLATFORM_FW_CHECK: FAIL … v<n>` | flashed from a different tree — `git pull`, re-flash |
| **UNKNOWN** | `unknown` | `PLATFORM_FW_CHECK: UNKNOWN` | **not** a stale flash: no RobotState read landed at all. Most often the **known benign boot-read transient** on a launch-only restart (the same miss that gives you a surprise re-home) — **relaunch once and re-read**; investigate CAN3/relay only if it repeats. Note the verdict does **not** self-heal within a launch: it is re-read only on a UDP reconnect or a CAN3 WARN/BUS_OFF→OK edge, neither of which a clean launch produces |
| **(no verdict)** | key absent | *(no line)* | **not a board state** — the running node predates the check, i.e. the `colcon build` was skipped. Rebuild both packages, source, relaunch. Never read an absent `FAIL` as a pass |

`robot_state.platform_fw_version` / `.platform_fw_version_read` carry the same
verdict typed, and land in the bag. Prefer `link_status` live: `ros2 topic echo`
gives false negatives for high-rate RELIABLE topics on this box and `robot_state`
runs at 100 Hz.

### THE TRAP: it reports, it does not protect

**Nothing refuses anything on a version skew.** Not a hand command, not a toss,
not a state transition. That is deliberate and the reason is specific: the
`SetHandTrajCmd` path carries the kind-3 retract, and a kind-3 clobbering an armed
kind-0 is the **only un-arm mechanism the Teensy offers** — a pre-release
`SAFE_ABORT` depends on it. A version gate there would convert a skipped flash
into "the abort stopped working with a ball about to launch". The full argument,
including why refusing kind-1 mid-sequence and why gating on a cached relay read
are both worse, is `ros_ws/docs/platform_fw_version.md` § Warn, never refuse.

So: **a `0 (PRE-VERSIONING)` reading invalidates every § CHECK HAND-4 row, and the
machine will happily let you run them.** You are the enforcement.

### The compile gate (INST-5)

`Teensy_code_platform/platformio.ini` builds the whole sketch for the Teensy 4.0
(`cd ros_ws/src/jugglebot/Teensy_code_platform && pio run`). Before 2026-07-27 nothing in
the repository compiled this sketch — `tests/firmware/test_hand_smooth_move_xref.py`
host-compiles `Trajectory.h` with `g++`, which is real coverage of that one header
but not of the `.ino` against FlexCAN_T4, SCL3300 and the generated config.

It is a **compile gate, not a flash path**: it has no `upload_command` and cannot
flash the board (PlatformIO's bundled loader helpers are glibc-2.34 and will not
run on this Jetson). **Keep flashing from the Arduino IDE.** The pio build uses
`-fno-exceptions` and a patched linker script, so its binary is not the image this
board has been running, and switching the flash toolchain in the same sitting that
validates the velocity-continuous `makeSmoothMove` would confound the two.

---

## Section LVL — `levelling-frame-contract` Phases 1–2 (one levelling frame)

> ### ⚠ SUPERSEDED IN PART, 2026-07-27 — this section describes the C-LEVEL-1-ONLY machine
>
> `catch-reach-degenerate-overshoot` Phases 2 and 3 landed **after** this section
> was written and changed three of its expectations. § CHECK CCATCH-3's banner
> flagged one of them; this banner covers all three, because they are stated *here*
> and a top-to-bottom reader meets them here first. Everything else in this section
> — LVL-0, LVL-1, LVL-2, LVL-3's park-plateau gate, LVL-4's parked-tilt gate — is
> **unchanged and still the authority**.
>
> | stated below | **actually correct now** |
> |---|---|
> | *"What did NOT land: the visible pre-throw tilt swing… it survives this fix at roughly its original size"*, and **pre-brief item 3** (`+2.9198°` at a 3.70 s lead, `0.789132°` per second of lead) | **The swing is GONE.** C-CATCH-1 stopped `build_catch` manufacturing an arrival twist for a gravity-level catch, so the reach is **flat / monotone** and commands no swing at all. Everything the pre-brief says about *why* the swing existed is still the correct explanation of the **pre-fix** data — read it as history |
> | **CHECK LVL-3's `peak_above_park` REPORT block** (expect `+2.9198°`, cross-check `implied lead ≈ 3.7 s`) | `peak_above_park` is now **≈ 0** on a healthy level catch, and `implied lead` is meaningless when the model's own input is zero. **Do not score either.** § CHECK CCATCH-2 replaces them: peak above park ≤ `1.05 ×` the requested displacement, monotone toward the target |
> | **CHECK LVL-3's `settle_rx` REPORT** (expect **−1.0784°**) and **CHECK LVL-4's** *"Platform tilt during the catch settle, expected 0.30° off level"* and *"peak tilt during the reach, expected ≈2.92°"* | Settle is now **exactly the commanded target** (`−0.7788 / −0.0692°` for the 2026-07-25 offset) and the residual vs gravity at contact is **0.0000°**, not 0.3008°. The mocap peak during the reach is the requested displacement, not `2.92°` |
> | **CHECK LVL-5**: *"REPORT: expect ≈16 mm, unchanged"* | The 16 mm **was** the 0.3008° through-seat residual, and C-CATCH-1 removed that residual. The criterion is now **CCATCH-2t: < 10 mm**. It is a **prediction, not a measurement** — nothing has yet measured a post-C-CATCH-1 catch error, so a reading between 10 and 16 mm is new information, not a contract failure |
>
> Pre-brief items **1** (the resting platform sits 0.78° off its own frame) and
> **2** (the first `go_home` after a `level` is a real 2.77 mm move) are
> **unaffected and still correct**.

**Plan**: `plans/parked/levelling-frame-contract.md` § Phases 1–2
**Contract**: `ros_ws/docs/levelling_frame.md` (**C-LEVEL-1**)
**What landed**: `trajectory_node` applied the gravity-levelling correction on
two of its six external pose-ingest surfaces and not the other four, so *"level"
meant two different things inside one node*. The toss's positioning
`go_to_pose` parked the platform at plan-frame `rx = 0` while the pre-tilt
`catch/dynamic_target` asked for plan-frame `rx = −0.7788°`, and `build_catch`
planned a min-jerk reach between the two frames. So on 2026-07-25 15:17:48 the
platform **rested 0.78° off gravity** and the two surfaces disagreed about where
level was. All six surfaces now pass through one shared `motion/levelling.py`, and
`mpc_bridge_node`'s verbatim second copy of the transform is deleted.

**What did NOT land**: the *visible* pre-throw tilt swing. That is `build_catch`'s
arrival twist, it survives this fix at roughly its original size, and pre-brief
item 3 below is the one thing to read before scoring anything here.
*(**⚠ No longer true as of `catch-reach-degenerate-overshoot` Phase 2 — the swing
is GONE.** See the SUPERSEDED banner at the head of this section. This paragraph is
the correct account of the machine as it stood when C-LEVEL-1 landed alone.)*

**Build needs**: **colcon + relaunch** (`trajectory_node.py`,
`mpc_bridge_node.py`, `motion/levelling.py`, `motion/trajectory/planner.py`
changed). **No firmware flash. No interface change *in this section*** — but build
`colcon build --packages-select jugglebot_interfaces jugglebot` anyway (the two-package build is mandatory in EVERY section since 2026-07-29 — `reload_coordinator_node` imports `TossContinuous` at module scope, so a `jugglebot`-only build raises `ImportError` before that node is constructed and takes `Reload`, `Toss` and `TossContinuous` down together; matrix row B).

> ### ⚠ OPERATOR PRE-BRIEF — read before the first post-fix launch
>
> Three things are **correct** and will look like faults if you are not
> expecting them. **Read #3 first — it is the one that would otherwise read as
> "the fix did nothing".**
>
> 1. **After `level`, the resting platform sits ~0.78° off its own frame.** It is
>    now level with respect to **gravity** at all times — at rest, at `go_home`,
>    between goals. Total tilt for the 2026-07-25 offset: **0.78185°**
>    (`rx −0.77878°`, `ry −0.06917°`). This is the fix working. Do not report it
>    as a new tilt fault.
> 2. **The first `go_home` after a `level` is a real move, not a no-op.**
>    ACTIVATE parks the legs at the IK of the *uncorrected* neutral, so `go_home`
>    to the corrected neutral walks the legs by
>    `[-1.42, +2.74, +2.77, -0.98, -1.35, -1.75] mm` — worst leg **2.7736 mm =
>    0.03908 rev** over the 2.0 s profile. That is 13 % of the pump's 0.3 rev
>    step gate as a whole-move excursion, 3.9 % of the firmware's 1.0 rev
>    MAX_DEVIATION band, and 322× the 8.6 µm leg encoder dead-band — so it IS
>    visible in `/leg_setpoint_echo`, which is exactly what makes CHECK LVL-2
>    measurable. Peak leg velocity **2.60 mm/s**; per-knot |Δu0| **9.2e-4 rev**.
>    Two hardware runbooks still describe this teardown `go_home` as "a genuine
>    no-op" (`session_phase1_hold.md`, `mvp_bench_runbook.md`) — they are
>    annotated, but if you are working from memory, expect the small move.
> 3. **⚠ NO LONGER TRUE — `catch-reach-degenerate-overshoot` Phase 2 REMOVED the
>    swing (see this section's SUPERSEDED banner). Read item 3 as the pre-fix
>    account; it is why the swing existed, and it is still the right explanation of
>    the 2026-07-25 data. Expect a FLAT reach now.**
>
>    **THE PRE-THROW TILT SWING IS STILL THERE. This fix does not remove it.**
>    You will still see the platform "slowly tilt back, then forward" before every
>    toss, and it will be roughly the same size as on 2026-07-25 — about
>    **+2.9°** at the ~3.7 s catch lead that session ran. That is **not** a
>    failure of this fix and **not** a reason to abort.
>
>    Why: `build_catch` specifies a non-zero *arrival twist* on its reach, so even
>    a reach that starts and ends at the same tilt has to swing out and come back.
>    The excursion is `(16/81) × 0.07 rad/s × |tdir_x| × lead` — for this offset
>    direction **0.789132° per second of catch lead**, exactly linear, and
>    **independent of how well the machine is levelled**.
>
>    What this fix actually changes is where the swing starts *from*. Pre-fix the
>    platform rested 0.7788° off gravity and peaked at **+3.0992°** against
>    gravity; post-fix it rests at **0°** and peaks at **+2.9198°**. A 0.18°
>    improvement in a 2.9° swing — plus the thing that actually mattered, a park
>    that is finally gravity-level and two ingest surfaces that finally agree.
>
>    Removing the swing is `plans/parked/catch-reach-degenerate-overshoot.md`.
>
> ### ⚠ AND ONE THING THIS FIX DOES **NOT** CLOSE — the criteria below are revised
>
> **⚠ AND THEN C-CATCH-1 DID CLOSE IT (2026-07-27 note).** Everything in this box
> is a correct account of the machine with C-LEVEL-1 alone, and it correctly
> explains the 16 mm. `catch-reach-degenerate-overshoot` Phase 2 then landed and
> removed the through-seat's wrong aim, so the 0.3008° residual is **0.0000°** and
> the 16 mm term is gone. The *first* criterion stays revised — "flat to ±0.05°
> across the whole goal" is still replaced by LVL-3's park plateau, for the separate
> reason that a park-plateau measurement is what actually tests frame agreement. The
> *second* is un-revised: **catch error < 10 mm is live again**, as
> § CHECK CCATCH-2's gated row. It is a prediction, not yet a measurement.
>
> The plan's Phase 4 predicted commanded `rx` **flat to ±0.05° across the whole
> goal** and a catch error **< 10 mm**. **Neither is reachable by this fix**, and
> the implementing session established why in closed form and confirmed it
> against the 2026-07-25 bag to **0.0009°**:
>
> `planner.build_catch` reads `catch_pose[3:5]` as "the receive tilt" and ramps a
> tilt-through-seat residual along it. That premise holds only while the
> commanded frame **is** the gravity frame. With a correction loaded, a
> *gravity-level* catch arrives as a non-zero plan-frame tilt — the correction
> itself — so the through-seat always engages and settles the platform
> `0.5 × 0.07 rad/s × 0.15 s = 0.005250 rad = ` **0.3008° off gravity-level, at
> ball contact**, in the correction's own direction. Closed form for the
> 2026-07-25 offset: **−1.078408° rx / −0.095775° ry**, against that bag's
> recorded **−1.0784 / −0.0958**. The un-levelled 15:04:35 baseline reads flat
> for the mirror reason: with no correction the level catch target has
> `tmag == 0`, which disables the through-seat entirely — so "match the
> un-levelled baseline" was never an achievable target once a correction is
> loaded.
>
> **Consequence**: the 0.3008° residual ⇒ **~16.5 mm** of lateral drift at
> 3.93 m/s over 0.8 s, which is the 16 mm catch error already measured. Expect
> the catch error to stay ≈16 mm after this fix. Closing it needs a change to
> `build_catch`'s through-seat aim, which alters commanded motion at ball
> contact on every catch including the shipping reload path — **an operator
> decision, deliberately not taken by the implementing session.**
>
> The *same* `build_catch` reach shaping also keeps the **visible pre-throw
> swing** — pre-brief item 3 above. Two consequences of one planner premise; both
> outside C-LEVEL-1.

### Pre-flight LVL-0 — confirm the freshly-built code is live

```bash
grep -q "levelling.correct_pose" \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/trajectory_node.py \
  && echo INSTALLED_OK || echo INSTALLED_STALE
grep -c "_apply_gravity_correction" \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/mpc_bridge_node.py
```

- **PASS**: prints `INSTALLED_OK`, then `0`.
- **ABORT**: `INSTALLED_STALE`, or a non-zero count on the second command — the
  *installed* copy predates the migration and every check below would score the
  old code. Rebuild and relaunch.

Then prove the **verdict instrument** itself still reads both shapes (read-only,
no robot, ~3 s):

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python tools/probes/levelling_tilt_bag_check.py --self-check
```

- **PASS**: `SELF-CHECK: PASS` — it scores a synthetic post-fix session PASS, a
  synthetic pre-fix session FAIL, and an ACTIVATE-contaminated session FAIL with
  the ambiguity note. Exit 0.
- **ABORT**: anything else. An instrument validated only against the *broken*
  shape will score a working fix as a failure and burn the sitting.

### CHECK LVL-1 — the `level` itself, and what got stored

Validates: that a correction actually reached `trajectory_node`. **Do this
first — every check below is vacuous without it**, and `/gravity_offset` is
published VOLATILE with one latched auto-push per orchestrator boot, so a
`trajectory_node` that restarted after the `level` holds an identity correction
while `RobotState.levelling_complete` still reads True (contract
§ *The correction can be silently absent* / **C-LEVEL-1.O**;
`levelling-frame-contract` Phase 3 closed the observability half — see
§ Section LVLGATE below, which is where that closure is scored).

Run the normal `level` routine, then:

```bash
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "Gravity offset published"
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "gravity correction set"
```

- **PASS**: both lines present, **after** the most recent launch, and the two
  `[tilt_x, tilt_y]` values are identical. Record them — CHECK LVL-3 needs them.
- **ABORT**: the orchestrator line appears but `trajectory_node`'s does not. The
  correction never landed in the node that applies it; nothing below is
  meaningful. Re-`level` after confirming `trajectory_node` is up.
- **ABORT**: no `gravity correction set` line at all **after the last relaunch**
  (a stale one from a previous launch does not count) — this is precisely the
  Phase-3 hazard, and it silently reverts the platform to the pre-fix frame.

### CHECK LVL-2 — the first `go_home` after `level` is a real, smooth, small move

> **⚠ SUPERSEDED 2026-07-27 by the run-sheet LVL-2 row — score against that, not
> against the bullets below.** The `2.77 ± 0.30 mm` PASS and the `> 5 mm` ABORT
> here are the numbers for the **2026-07-25 offset only** (`0.013592, 0.001207`
> rad = 0.78185°). Both scale with your measured offset:
> `predicted_mm ≈ 203 × hypot(tilt_x, tilt_y)` in **radians**. At 1.41° of bench
> tilt a *healthy* machine reaches 5.0 mm and the fixed ABORT below would fire on
> correct behaviour at the first actuating move of the sitting. The bullets are
> kept as the reference-offset instance of the run-sheet form.
>
> **The ABORT is now `max(1.8 × predicted_mm, 2.0 mm)`, and the two arms catch
> different things.** The `1.8 ×` arm is the **double-application detector** — the
> mirror bug C-LEVEL-1's second half exists to prevent, where an FK-derived seed
> that is *already* in the corrected frame gets corrected again. Double
> application produces exactly `2 × predicted`, so an arm at `1.8 ×` catches it
> with margin **at every offset where that arm binds**. The `2.0 mm` floor exists
> only so a near-zero offset does not abort on encoder noise (it is ~230× the
> 8.6 µm leg encoder dead-band), and it is the operator's number: 2 mm of platform
> travel is rarely enough to decide a catch.
>
> Know what that buys and what it does not. The floor binds below **0.313°** of
> offset; above that the detector arm binds. Double application is therefore
> caught for any offset above **0.282°** — against 0.705° under the old fixed
> 5 mm, so 2.5× more of the range is covered. Below 0.282° a doubled correction is
> ≤ 2.0 mm of leg travel, i.e. below the level the floor was deliberately sized to
> ignore; that residual blind spot is accepted on the same reasoning that set the
> floor. At the 2026-07-25 offset (0.78185°) the arm binds at 4.99 mm and the
> floor never applies, so nothing about scoring that session changes.

Validates: Phases 1–2 ingest E5 (`go_home` targets the *corrected* neutral) and
the operator pre-brief above. Do this **before** loading a ball.

`level` → activate → TRAJECTORY → `trajectory/go_home`, with the § Recording bag
running.

```bash
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "returning to neutral"
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "peak_leg_vel_mmps\|move_seq"
```

- **PASS**: the platform makes a small, smooth, single move; `/leg_setpoint_echo`
  shows a worst-leg excursion of **2.77 ± 0.30 mm** (`0.0391 ± 0.0042 rev`) and
  the realized peak leg velocity on `/trajectory/diagnostics` is
  **2.60 ± 0.50 mm/s**. No pump rejection, no guard latch.
- **PASS (equivalent)**: if `level` reported a **zero** offset, this move is a
  genuine no-op — a legitimately level machine has nothing to correct. Record
  which case applied.
- **ABORT**: any leg moves **> 5 mm**, any step rejection in the bridge log, any
  `MAX_DEVIATION` or guard E-STOP. The correction magnitude is bounded by the
  offset; a large move means the correction is being applied more than once.
- **REPORT, do not abort**: the move is *visible but smaller than 2.4 mm*. The
  numbers above are for the 2026-07-25 offset (`0.013592, 0.001207 rad`); a
  different offset scales them roughly linearly. Scale before scoring.

### CHECK LVL-3 — the frames agree across a toss goal (**the headline check**)

Validates: Phases 1–2, the whole contract. **This is the analysis command that
turns the capture into a verdict.**

Run **one** toss with the § Recording bag enabled and
`tests/hardware/toss_trace_recorder.py record` running (see § Section HAND for
the recorder invocation — one capture scores both sections).

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/levelling_tilt_bag_check.py \
  --offset <TILT_X> <TILT_Y> --t0 <AFTER_THE_LVL-2_GO_HOME> --json
```

(`<TILT_X> <TILT_Y>` are the **radians** from CHECK LVL-1's log line. Defaults to
the newest bag under `~/Desktop/rosbags`; add `--bag <dir>` to pick one. ~5 s for
a 35 s window, ~2 min for a whole session.)

**`--t0` is not optional here.** Between ACTIVATE and the first `go_home` the node
holds at the FK of the *uncorrected* activate pose — commanded `rx ≈ 0`, which is
**correct** (a seed is never corrected: C-LEVEL-1's second half). If that idle
out-lasts the time actually parked at the corrected neutral — trivially easy while
you read this and fetch a ball — it wins the "longest plateau" vote and a perfectly
healthy session reads `park_rx ≈ 0`, i.e. exactly like the pre-fix frame. Set
`--t0` to a few seconds after the LVL-2 `go_home` and the ambiguity is gone. (The
probe prints a `NOTE:` when it detects this shape, but do not rely on it.)

- **PASS**: banner reads `VERDICT: PASS` — the **park** plateau (the longest one,
  where the platform rests between goals) is within **±0.05°** of
  `(−tilt_x, −tilt_y)` in degrees, on **both** axes. Exit code 0. **This is the
  only gated quantity in this check.**
- **ABORT**: `VERDICT: FAIL` with `park_rx ≈ 0.0000°` **and no `NOTE:` line**.
  That is the pre-fix frame — the installed code is not the migrated code (re-run
  LVL-0) or the correction never loaded (re-run LVL-1). If a `NOTE:` line IS
  printed, re-run with a later `--t0` before concluding anything.
- **ABORT**: `park_rx` outside ±0.05° in any *other* way — e.g. ≈ **−1.5576°**,
  which is exactly **twice** the correction and means the correction is being
  applied twice (the mirror bug: a derived/FK-seeded pose routed through the
  correction).
- **⚠ SUPERSEDED 2026-07-27 — DO NOT SCORE THE NEXT TWO BULLETS.** `peak_above_park`
  and `settle_rx` both described the C-LEVEL-1-only machine. C-CATCH-1 removed the
  arrival twist that produced both, so on a healthy capture `peak_above_park` is now
  **≈ 0** (not `+2.9198°`) and `settle_rx` is **exactly the commanded target** (not
  `−1.0784°`). `implied lead` is meaningless once the model's own input is zero.
  § CHECK CCATCH-2 carries the replacements. The two bullets are kept verbatim
  because they remain the correct account of the **pre-fix** bag, which is what
  CATCH-2 reproduces.
- **REPORT, never gate**: `peak_above_park`. Post-fix this equals the platform's
  **physical peak tilt against gravity**, and it is *larger* than the pre-fix
  reading of the same name — pre-fix the park itself sat 0.7788° high, so
  subtracting it hid part of the swing. Like-for-like, in the gravity frame: the
  reference session peaked at **+3.0992°**, and a healthy post-fix session at the
  same 3.70 s catch lead reads **+2.9198°**.

  There is **no threshold** on this number, and the earlier "at or above +3.099°
  ⇒ ABORT" line is **retired**: `peak_above_park = 0.789132° × (catch lead in s)`
  exactly, so any fixed threshold is really a threshold on the lead. That one
  would have fired on a perfectly healthy system at any lead ≥ 3.93 s and sat only
  0.18° away at the lead the reference session ran.

  What to read instead: the probe prints `implied lead`, the catch lead the
  arrival-twist model infers from the peak. On a healthy post-fix session the model
  is exact, so this recovers the toss's real catch lead — expect **≈3.7 s** for the
  same toss geometry the reference session ran, and cross-check it against the
  reach lead you can read off the probe's own plateau table (the gap between the
  park plateau ending and the settle plateau starting, which under-reads the true
  lead by ~0.2–0.4 s because plateau edges are detected with a 0.02° band). A
  *materially short* implied lead is the pre-fix signature: that bag reads **2.94 s
  against a true 3.70 s**, because a frames-disagree reach carries an extra
  `−0.7788·h(s)` term the model does not know about. This is corroboration, not a
  gate — the park plateau is the gate.
- **REPORT, do not gate**: `settle_rx`, expected at
  `park_rx × (1 + 0.005250/|offset|)` = **−1.0784°** for the 2026-07-25 offset.
  This is the through-seat overshoot described in the pre-brief. It is **not**
  closed by this fix, and a `settle_err` near zero confirms the model rather
  than indicating a fault.

**Do NOT score this section against "commanded `rx` flat to ±0.05° across the
whole goal".** That criterion, as written in the plan's Phase 4, cannot be met
while `build_catch` aims its through-seat residual off the plan-frame tilt — see
the pre-brief. The park-plateau criterion above replaces it and measures the same
thing (do the positioning and catch surfaces agree on what "level" means)
without depending on a defect this plan does not own.

### CHECK LVL-4 — mocap cross-check: the platform is physically level

Validates: Phases 1–2 against an instrument that does **not** share the FK path.

> ### ⚠ SUPERSEDED 2026-07-27 — LVL-4 IS NO LONGER A GATE. Its ABORT signature is what a CORRECT machine produces.
>
> Two defects, both found by building the instrument this row had never had:
>
> **1. There was no instrument at all.** `grep -rn rigid_body_poses tools/probes/`
> returns **nothing** — no probe in the repo reads the topic. The row carried a
> `±0.10°` number and a "stop the sitting" ABORT with no documented way to produce
> the number, so at the bench it would be skipped or hand-rolled under time
> pressure. A validated reader is supplied below.
>
> **2. `Base` is not a gravity reference, so the ±0.10° premise is inverted.** The
> row assumed the mocap `Base` body is level, hence that a levelled Platform reads
> `≈ 0°` against it. Measured on the pre-fix reference bag
> (`~/Desktop/rosbags/2026-07-25_15-17-48`, run 2026-07-27 with the reader below):
> parked Platform-vs-`Base` tilt is **0.0869° median over the whole bag, 0.132°
> median over the quiet 5–40 s window** — i.e. the platform sits essentially *on*
> the `Base` body's frame. At that same physical state the inclinometer measured
> **0.782°** off gravity (the bag's own `/gravity_offset`:
> `hypot(0.013592, 0.001207) = 0.013646 rad`; the `tilt_x` component alone is
> `0.7788°`). Both cannot be level: **`Base` is ~0.78° off gravity**, and its
> local frame is a QTM body-definition artefact, not a plumb line.
>
> The correction moves the platform *physically* — that is the whole point of
> "a commanded `−0.78°` **is** physically level" — so a correctly-levelled
> post-fix park is expected to read **≈ 0.78° against `Base`**, which is verbatim
> the old ABORT ("physically tilted by ~0.78° while parked ⇒ stop the sitting").
> Scoring it as written aborts a working machine, exactly as § CHECK CCATCH-3's
> `peak off the park` row did. **LVL-4 is now REPORT-only.** The gated levelling
> verdict is § CHECK LVL-3, which is instrumented, and whose own three-way
> discrimination (`≈ 0` / correct / `≈ 2×`) this row can only corroborate.

**The reader (validated 2026-07-27 on the reference bag: 46 161 samples over
290.3 s, ~53 s wall-clock, 0 failures).** Venv, standing rule 5. `world →
platform_start` is a **translation-only** static TF (`mocap_node.py:195-203`,
`rotation.w = 1.0`), so the two bodies' quaternions are directly comparable.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python - "$BAG" <<'PY'
import glob, sys, math
import numpy as np
from mcap_ros2.reader import read_ros2_messages
def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                     [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
rows = []
for path in sorted(glob.glob(sys.argv[1] + '/*.mcap')):
    for m in read_ros2_messages(path, topics=['/rigid_body_poses']):
        d = {b.name: b.pose.pose.orientation for b in m.ros_msg.bodies}
        if 'Base' not in d or 'Platform' not in d:
            continue
        qb = (d['Base'].x, d['Base'].y, d['Base'].z, d['Base'].w)
        qp = (d['Platform'].x, d['Platform'].y, d['Platform'].z, d['Platform'].w)
        if any(math.isnan(v) for v in qb + qp):
            continue           # Catching_Cone drops out as NaN; Base/Platform can too
        zp = (qmat(qb).T @ qmat(qp)) @ np.array([0.0, 0.0, 1.0])
        rows.append((m.log_time_ns * 1e-9,
                     math.degrees(math.acos(max(-1.0, min(1.0, float(zp[2]))))),
                     math.degrees(math.atan2(zp[1], zp[2])),
                     math.degrees(-math.atan2(zp[0], math.hypot(zp[1], zp[2])))))
a = np.array([[r[1], r[2], r[3]] for r in rows])
print('samples %d over %.1f s' % (len(rows), rows[-1][0] - rows[0][0]))
print('tilt_deg median %.4f  p95 %.4f  max %.4f'
      % (np.median(a[:, 0]), np.percentile(a[:, 0], 95), a[:, 0].max()))
print('rx_deg   median %+.4f  min %+.4f  max %+.4f' % (np.median(a[:, 1]), a[:, 1].min(), a[:, 1].max()))
print('ry_deg   median %+.4f  min %+.4f  max %+.4f' % (np.median(a[:, 2]), a[:, 2].min(), a[:, 2].max()))
PY
```

Narrow it to the park by adding a bag-relative time filter on `rows` — zero-based
on this reader's **own** first `/rigid_body_poses` sample (`t − rows[0][0]`), which
is **not** the same origin as `--t0`. `--t0` zero-bases on the first
`/leg_setpoint_echo` sample, and QTM streams continuously while
`/leg_setpoint_echo` stays silent until the pump accepts a setpoint — so the two
origins differ by however long arming took. Carrying a `--t0`-derived window across
to this reader selects an earlier span, plausibly the pre-`activate` idle, where the
*uncorrected* Platform reads 0.087° — which is the signature this check calls "the
correction never reached the legs". See § The `--t0/--t1` origin note.

- **REPORT (the headline)**: parked Platform-vs-`Base` tilt. Against the pre-fix
  baseline of **0.087°**, expect it to have moved **BY the correction**, i.e. to
  read **≈ 0.78°** (`hypot(tilt_x, tilt_y)` from LVL-1). Reading **≈ 0.087°
  unchanged** says the correction never reached the legs — cross-check LVL-3,
  which gates that. Reading **≈ 1.56°** (twice the correction) is the
  double-application signature LVL-3 also names; that one **is** worth stopping
  for, but confirm it on LVL-3 first, because this row's absolute zero is an
  unmeasured constant and LVL-3's is not.
- **Sanity check the reader itself, in the same output**: `ry_deg max` should
  land near the reload pre-tilt (reference bag: **+11.33°** against a commanded
  `−10.64°` settle — magnitude and axis match, sign is this reader's own `atan2`
  convention, not a fault) and `rx_deg` should stay inside roughly `[−3.7, +1.5]°`.
  If those are wrong the frames are wrong and nothing else here means anything.
- **⚠ SUPERSEDED 2026-07-27 — the next two REPORT bullets described the
  C-LEVEL-1-only machine.** With C-CATCH-1 in force the mocap should read
  **≈ 0° off level at the catch settle** (not `0.30°`) and a reach peak equal to the
  **requested displacement** (not `≈2.92°`). Kept verbatim as the correct account of
  the pre-fix bag.
- **REPORT**: the Platform tilt during the catch settle, expected **0.30°** off
  level (the through-seat overshoot).
- **REPORT**: the Platform's peak tilt during the catch reach, expected
  **≈2.92°** off level at a 3.70 s catch lead (`0.789132° × lead`). This is the
  mocap-side confirmation of pre-brief item 3 — the swing is still there, and it
  is *smaller* than the pre-fix `+3.0992°` by exactly the correction's
  contribution at the peak. Not a gate.

### CHECK LVL-5 — catch error (expectation-setting, not a gate this round)

> **⚠ SUPERSEDED 2026-07-27 by § CHECK CCATCH-2's tracker-catch-error row
> (run-sheet id CCATCH-2t).** The `≈16 mm` below **was** the 0.3008° through-seat
> residual — `0.005250 rad × 3.93 m/s × 0.8 s = 16.5 mm` — and C-CATCH-1 removed
> that residual, so the criterion is now **< 10 mm** and it *is* gated (by
> CCATCH-2). Two honest caveats, because the correction runs the other way from the
> usual: **< 10 mm is a prediction, not a measurement** — nothing has yet measured a
> post-C-CATCH-1 catch error on hardware — and the bullet below that calls a reading
> "materially below 16 mm" a *surprise* is now inverted: it is the **expected**
> result. A reading between 10 and 16 mm is new information about what else
> contributes, not a failure of either contract.

Validates: nothing this plan closes — recorded so the number is on file for the
`build_catch` through-seat decision.

- **REPORT**: the tracker-measured catch error. **Expect ≈16 mm, unchanged.**
  The plan's `< 10 mm` target is not reachable by this fix (see the pre-brief);
  the 16 mm is the 0.3008° through-seat residual, not the frame plumbing.
- **REPORT**: judge the catch **by eye** as well. *(Updated 2026-07-28 — this
  read "tracker verdicts still read MISSED on real catches (the Phase-7 reload
  arc)", which is now only half true and the wrong half would make you discard a
  real finding.)* This row is a **self-toss**, so since C-POSSESS-1 its `outcome`
  is expected to read `CAUGHT` on a real catch — a `MISSED` here is a **finding**,
  not expected noise. Reload verdicts ~~do still read `MISSED`~~ **read `CAUGHT` on
  a real catch too since 2026-08-10** — the hand ball sensor is the primary source
  and does not care about the split track (standing rule 3, rewritten). Row
  **POSS-1** is where the counts go.
- **A catch error materially *below* 16 mm would be a surprise** and is worth
  capturing carefully: it would mean the through-seat model is not the dominant
  term after all.

### Not in this section

- **`REJECTED_NOT_LEVELLED`** is `levelling-frame-contract` **Phase 3**. It
  LANDED 2026-07-26 and has its own section — **§ Section LVLGATE**. Do not
  score it here; do note that it changes this section's own preconditions, since
  a toss issued before `level` now refuses instead of running.
- The **hand dip** is `hand-command-continuity` (§ Section HAND). If both have
  landed, validate them in one sitting but score them separately.

---

## Section CATCH — `catch-reach-degenerate-overshoot` (catch-reach excursion)

> **Phase 0/1 (landed 2026-07-26) need NO powered sitting.** They are offline and
> read-only, and their gate — the excursion reproduced from recorded inputs — is
> already met against `~/Desktop/rosbags/2026-07-25_15-17-48`. Everything in this
> section is either (a) an analysis command you can run on any capture without
> touching the robot, or (b) the shape Phase 2's sitting will need **once the
> operator has re-prioritised it** (Phase 1's STOP condition FIRED — see the plan).
>
> **Do not start Phase 2's sitting from this section alone.** Phase 2 changes
> commanded motion at ball contact on *every* catch, including the shipping reload
> path, and its invariant (C-CATCH-1) is not yet ratified.
>
> **⚠ UPDATE 2026-07-27: Phase 2 AND Phase 3 have both landed** (`407154f`,
> `e58ed89`), so "not yet ratified" is history and "Phase 2's sitting" is **this**
> sitting. Its shape is not derived from this section: it is § THE RUN SHEET at the
> top of this file, stage 6. The instruction above still holds in the form that
> matters — do not start from this section alone.

**Prerequisites for CATCH-1/2 (analysis only):** none beyond the venv. No colcon
build, no relaunch, no firmware flash. Nothing here actuates the machine — these
commands open `.mcap` files and import pure-Python planner modules.

**Prerequisites when Phase 2 eventually lands (CATCH-3):** `colcon build
--packages-select jugglebot_interfaces jugglebot` **and a relaunch** (the launch
runs the *installed* copy) (the two-package build is mandatory in EVERY section since 2026-07-29 — `reload_coordinator_node` imports `TossContinuous` at module scope, so a `jugglebot`-only build raises `ImportError` before that node is constructed and takes `Reload`, `Toss` and `TossContinuous` down together; matrix row B). **No firmware flash** — nothing in
this plan touches the Teensy.

### CHECK CATCH-1 — instrument health (run FIRST, no bag, no robot)

Validates: `catch-reach-degenerate-overshoot` Phase 0's harness itself.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py --self-check
```

- **PASS**: `SELF-CHECK: PASS`, 8/8 `OK` lines, exit 0.
- **ABORT**: any `BAD ` line. In particular a BAD on *mirrored production
  constants have not drifted* means the planner's constants moved and the probe's
  closed form is stale — **do not score a session with it** until the mirrored
  values in `catch_reach_replay.py` are re-derived. This is the expected failure
  the first time Phase 2 lands, and it is deliberate.

### CHECK CATCH-2 — score a session's catch reaches (read-only, any capture)

Validates: `catch-reach-degenerate-overshoot` Phase 0 (reproduction) on new data.

Recording: `record:=true` (§ Recording). The topics this section scores —
`/trajectory/status /trajectory/diagnostics /trajectory/target_feedback
/catch/dynamic_target /gravity_offset /throw_announcements /leg_setpoint_echo` —
are all in the ONE list as of 2026-08-10; nothing to append.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
# map the prose's "toss #N" onto --toss: ONE table, all throwers, both indices
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> --list
# score one reach (repeat per row; --thrower selects the column)
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> \
    --toss N --json --csv
```

PASS/ABORT, as numbers the verdict actually enforces:

| Row | PASS | ABORT |
|---|---|---|
| `VERDICT` | `REPRODUCED`, exit 0 | `NOT-REPRODUCED`, exit 1 |
| rx **and** ry `max` residual | ≤ `max(0.05°, 0.01 × commanded span)` — printed as `tolerance ±X` | above it |
| fitted `echo lag` | `0 < lag ≤ 25.0 ms` (one knot) | negative (unphysical — the echo cannot precede the command) or > 25 ms |
| `SINGLE INSTALL` | `True` | `False` — read the census rows to see which clause fired |
| `/gravity_offset` line | absent, or `1 distinct value` | `RE-LEVEL in this session` — re-check which offset each reach was scored against |
| `fk_failures` (JSON) | `0` | any |

A `NOT-REPRODUCED` is **a publishable result, not a probe bug** — read the
FEATURE LEDGER before blaming the harness. But run CATCH-1 first: a stale
instrument produces exactly this reading on a healthy system.

### CHECK CATCH-3 — the pre-fix baseline to compare a Phase-2 capture against

Validates: nothing on its own. These are the reference numbers, measured
2026-07-26 on the already-captured `2026-07-25_15-17-48` — no sitting needed.

```bash
python tools/probes/catch_reach_replay.py \
    --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --toss 2
python tools/probes/catch_reach_replay.py \
    --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --thrower ball_butler --toss 2
```

| quantity | pre-fix value | what a correct Phase-2 fix does to it |
|---|---|---|
| toss commanded `rx` peak | `+2.3224°` from a `−0.778784°` target | **must shrink toward the target** — this is the defect |
| toss settle `rx` | `−1.0784°` (target × 1.38473) | must approach `−0.7788°` (× 1.000) |
| toss amplification | `3.756` | must drop below `0.790` (the sign-reversal threshold) |
| reload settle | `+1.8235 / −10.9330°` (target × 1.0279) | **WILL change** — a fix resizes or removes the through-seat. This is a pre-fix **reference, NOT a post-fix pass criterion** |
| reload catch accuracy | as flown | must be **no worse** |
| tracker catch error | ≈16 mm (see § CHECK LVL-5) | target `< 10 mm` per `ros_ws/docs/levelling_frame.md` § "The 16 mm" |

### When Phase 2's sitting happens — minimum shape

Not a check yet; recorded so it is not re-derived under time pressure.

1. ~~**Reboot the can-bridge Teensy** before the session (standing session rule).~~
   Retired 2026-08-15 — see standing rule 1. Record `uptime_ms` as always.
2. `level` is per-boot — a manual `level` is **always** required first.
3. Record with `record:=true` — the ONE list (§ Recording) already carries the
   six catch topics above as of 2026-08-10.
4. Score with CATCH-1 → CATCH-2 → CATCH-3, then
   `tools/probes/levelling_tilt_bag_check.py --offset <TILT_X> <TILT_Y> --t0
   <after the first go_home>` for the park (§ CHECK LVL-3's instrument).
5. Judge catches **by eye** as well as by `outcome`, and score the two paths
   differently — see standing rule 3. **self-toss**: `outcome` is expected to read
   `CAUGHT` on a real catch since C-POSSESS-1, so a `MISSED` is a **finding**.
   **reload**: still reads `MISSED` on a real catch, correctly.
   *(Updated 2026-07-28; this read "tracker verdicts still read MISSED on real
   catches" without qualification.)*

### Not in this section

- The `peak_leg_*` predicted-peak staleness (six install paths bump `move_seq`
  without writing the field) is **diagnosed and annotated, not fixed** — see
  `tests/hardware/mvp_bench_runbook.md` open item 7. It is a diagnostics-reading
  hazard, not a motion defect, and needs no bench check.
- The park frame and `go_home` are § Section LVL. If both plans have landed,
  validate them in one sitting but score them separately.

---

## Section CCATCH — `catch-reach-degenerate-overshoot` Phase 2 (contract C-CATCH-1)

> **Appended 2026-07-26, when Phase 2 landed.** This section supersedes three
> statements made earlier in this file. They are left in place (sections are
> append-only here) — read these three corrections first or you will score a
> healthy machine as broken:
>
> 1. **§ CHECK CATCH-1** says "8/8 `OK` lines". The self-check now has **ten**
>    cases (two were added by Phase 2: the two-sided C-CATCH-1 counterfactual and
>    the bound-factor derivation). PASS is still `SELF-CHECK: PASS` + exit 0.
> 2. **§ CHECK CATCH-3**'s toss rows are the **pre-fix** reference, and its
>    `0.790` figure is **wrong in the permissive direction**. The sign-reversal
>    threshold is `40/81 = 0.4938`, not `ψ(2/3) = 0.790` (that is where the value
>    *at* `s = 2/3` crosses zero, not where the reach first leaves the park the
>    wrong way) — a gate sized off 0.790 passes a reach that has already
>    reversed. Post-fix the toss settle is `−0.778784°` (× 1.000) and the
>    amplification question is moot anyway: the through-seat does not engage at
>    all for a level catch. Its reload row is right and is the one to watch: the
>    reload settle **does** move, by the numbers in CCATCH-3 below.
> 3. **§ Section LVL**'s operator pre-brief ("the visible tilt REMAINS", the
>    `+2.92°` swing, `peak_above_park`) described the C-LEVEL-1-only machine.
>    Post-fix the catch reach is **flat / monotone**: a level catch commands no
>    swing at all. `ros_ws/docs/levelling_frame.md` carries the same correction.
>
> Also now fixed, and no longer "not in this section": the `peak_leg_*`
> predicted-peak staleness (`tests/hardware/mvp_bench_runbook.md` open item 7).
> It has its own check, CCATCH-5, and its own commit.

**What changed in the machine.** `planner.build_catch` used to read
`catch_pose[3:5]` as "the receive tilt" and ramp a tilt-through-seat residual
along it. With a levelling correction loaded that vector is the *correction*, not
the ball — so a gravity-level catch got an arrival twist nobody asked for, swung
`+2.32°` the wrong way mid-flight, and settled `0.3008°` off gravity **at ball
contact**. `build_catch` now takes the gravity-referenced receive tilt as its own
argument (contract **C-CATCH-1**, `ros_ws/docs/catch_arrival_contract.md`) and
bounds every departure from the target that an arrival twist it *derives* creates
— during the reach and during the settle — to `40/81` of the catch's physical
tilt **scale** (the larger of the requested displacement and the receive-tilt
magnitude). The reload's seat is untouched by that bound, at the pre-tilt install
**and** at every on-pose supersede through ball contact; what moves on the reload
is the seat's *aim*. *(Correction, 2026-07-26: that last clause held for the two
days C-CATCH-1 shipped with a `0.07` rate. The operator has since set the
manufactured rate to `0.0`, so what moves on the reload is the seat's aim **and
then the seat itself** — see § Section ZSEAT. The bound sentence stays true: it is
not the bound that removed the seat.)*

**Prerequisites:** `cd ~/Desktop/Jugglebot/ros_ws && colcon build
--packages-select jugglebot_interfaces jugglebot && source install/setup.bash`,
then **relaunch** (the two-package build is mandatory in EVERY section since 2026-07-29 — `reload_coordinator_node` imports `TossContinuous` at module scope, so a `jugglebot`-only build raises `ImportError` before that node is constructed and takes `Reload`, `Toss` and `TossContinuous` down together; matrix row B) —
the launch runs the *installed* copy, and this change is in `trajectory_node.py`
and `motion/trajectory/planner.py`. **No firmware flash**; nothing here touches
the Teensy. Standing session rules still apply — as amended: the bridge reboot is
**retired** (standing rule 1, 2026-08-15), and you still **check**
`gravity_correction_loaded` — `level` only if it reads `false` (standing rule 2,
corrected 2026-07-27).

### CHECK CCATCH-1 — instrument health (run FIRST, no bag, no robot)

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py --self-check
```

- **PASS**: `SELF-CHECK: PASS`, **10/10** `OK`, exit 0.
- **ABORT**: any `BAD`. Specifically:
  - *mirrored production constants have not drifted* → a planner constant moved
    after this section was written; re-derive the mirrors before scoring anything.
  - *C-CATCH-1 counterfactual is two-sided* → either the probe can no longer
    rebuild the pre-fix plan (so it cannot reproduce any older capture) or the
    fixed path is not actually removing the excursion. Do not run the sitting.

### CHECK CCATCH-2 — the level catch commands NO swing (**the headline check**)

Validates: C-CATCH-1 on the self-toss path. Run a normal self-toss goal with
`record:=true` — the ONE list (§ Recording) carries `/trajectory/status
/trajectory/diagnostics /trajectory/target_feedback /catch/dynamic_target
/gravity_offset /throw_announcements` as of 2026-08-10.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> --list
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> \
    --toss N --json --csv
```

Read the **C-CATCH-1 COUNTERFACTUAL** block it prints, and the FEATURE LEDGER
above it. For a *post-fix* capture the recorded reach should already look like
the counterfactual's "fixed" column, so the two collapse together.

| quantity | PASS | ABORT |
|---|---|---|
| commanded `rx` across the pre-tilt reach (FK of `/leg_setpoint_echo`) | **monotone** toward the target; peak above park ≤ `1.05 ×` the requested displacement | any excursion **away** from the target > `0.05°` — C-CATCH-1 is not in force (wrong binary? colcon + relaunch skipped?) |
| toss settle `rx` / `ry` | equals the commanded target to **±0.05°** (for the 2026-07-25 offset: `−0.7788 / −0.0692°`) | `−1.0784 / −0.0958°`, i.e. still `× 1.3847` — the old aim is live |
| residual vs gravity at ball contact | **≤ 0.05°** | ≥ 0.25° (the pre-fix value was 0.3008°) |
| plan segments for a level catch | **2** (reach + quiescent hold) | 3 — a decay segment means the seat re-engaged off a non-gravity frame |
| `peak_leg_acc_mmps2` / `_jerk_mmps3` on `/trajectory/diagnostics` for a toss catch install | `≈ 1.2 / ≈ 3` (was `142.4 / 3950`) | still `≈142 / ≈3950` |
| `VERDICT` | `REPRODUCED`, exit 0 | `NOT-REPRODUCED` — run CCATCH-1 first |
| tracker catch error (§ CHECK LVL-5's instrument) | **< 10 mm** | ≥ 16 mm — the improvement did not land |

**Judge catches by eye as well as by `outcome`** — as everywhere else in this
file. *(Updated 2026-07-28; this read "The tracker still reports `MISSED` on real
catches, so …".)* The row above is a **self-toss** (`CCATCH-2t`), and since
C-POSSESS-1 its `outcome` is expected to read `CAUGHT` on a real catch: a `MISSED`
on a catch you watched land is a **finding**, and routes to § SECTION POSS. ~~Reload
catches still read `MISSED`, correctly.~~ **Since 2026-08-10 a reload catch reads
`CAUGHT` too** (hand ball sensor primary) — standing rule 3 has the rewritten split.

### CHECK CCATCH-3 — the reload path CHANGED, on purpose (regression watch)

> **⚠ SUPERSEDED 2026-07-26 by § Section ZSEAT — FIVE of the rows below now ABORT
> on CORRECT behaviour, and a sixth has gone stale.** The operator set the
> manufactured seat rate (`planner._CATCH_TILT_THROUGH_RATE_RADPS`) to **0.0**, so
> the reload's through-seat no longer exists at all. Corrections, row by row, so a
> top-to-bottom reader cannot act on the stale table:
>
> | row | table below says | **actually correct now** |
> |---|---|---|
> | segment count | `3 → 3`, ABORT if 2 | **2** — reach + quiescent hold. 3 would mean the zero default did not land |
> | seat rate at ball contact | `0.070000` unchanged, ABORT below `0.0665` | **`0.000000 rad/s`** — a parked rim is the intended state |
> | settle `rx` / `ry` | `+1.844635 / −10.928741°` | **`+1.774062 / −10.636334°`** = exactly the commanded target (a further `−0.070573 / +0.292407°`) |
> | **`peak off the park`** | `10.9287°`, **ABORT if the delta exceeds `0.10°` in either direction** | **`10.6363°`** — a further **`−0.292407°`**, i.e. **2.9× that row's own ABORT threshold on a HEALTHY capture.** The row is the ry-axis peak, so it moves by exactly the ry settle delta above; the reach is monotone to the target with no overshoot past it (measured `0.000000°` past target, vs `0.292407°` at the `0.07` rate). Without this correction the table's closing paragraph sends the operator to "the receive tilt being threaded through is not the wire orientation" — a fault hunt on a working machine |
> | predicted acc / jerk | `142.0 / 3935` | **`37.9 / 170`** (and vel `23.8 → 29.0` — it goes *up*; see ZSEAT-3) |
> | the "PASS / ABORT" paragraph under the table | ABORT if the last 0.8 s is **flat** | **flat is the PASS** — inverted outright |
>
> **STALE rather than false — `arrival-rate bound`.** That row's ABORT is "the probe
> prints `BINDS`". At a `0.0` default the probe cannot print `BINDS` at all: it
> prints `MANUFACTURES NOTHING` (full text amended 2026-07-28 — match on that
> fragment, not on the parenthetical), and the `0.20004 rad/s` bound itself is still computed and still
> correct. So the row can never fire and never PASS as written — read it as
> informational, and see § CHECK ZSEAT-3, which quotes the new wording.
>
> The rest of the table — the aim rotation (still `4.0997°`, still scoreable, the
> seat's *aim* is unchanged by the zero default) and the `N further catch install(s)`
> census line — is unaffected and still reads as written. Score the reload against
> § Section ZSEAT, not against this table.

Validates: that the intended, quantified change on the **shipping** reload path is
the one that actually happened — and nothing larger.

The reload catch has a real receive tilt (10.87° on the wire for the 2026-07-25
capture), so its through-seat **survives**. What moves is its *aim*: from the
plan-frame tilt to the gravity-referenced one. Measured 2026-07-26 offline
through `tools/probes/catch_reach_replay.py` on
`~/Desktop/rosbags/2026-07-25_15-17-48 --thrower ball_butler --toss 2`
(lead 2.3712 s, offset `[0.013592347, 0.001207157]`):

| quantity | pre-fix | post-fix | delta | ABORT if |
|---|---|---|---|---|
| seat aim direction | plan-frame tilt | gravity receive tilt | **rotated 4.0997°** | > 6° — the wrong tilt is being threaded through |
| settle `rx` | `+1.823550°` | `+1.844635°` | **+0.021086°** | \|delta\| > 0.10° |
| settle `ry` | `−10.933038°` | `−10.928741°` | **+0.004297°** | \|delta\| > 0.10° |
| peak off the park | `10.9330°` | `10.9287°` | `−0.0043°` | > 0.10° in either direction |
| predicted `peak_leg_acc_mmps2` | `139.7` | `142.0` | +1.6 % | > 200 (session ceiling 5000) |
| predicted `peak_leg_jerk_mmps3` | `3873` | `3935` | +1.6 % | > 5000 (session ceiling 30000) |
| arrival-rate bound `2.5·scale/T` | — | `0.20004 rad/s` vs the `0.07` default | **does not bind** | the probe prints `BINDS` — the seat has been throttled on a real receive tilt; re-read the scale |
| segment count | 3 | 3 | unchanged | 2 — the seat was lost entirely |
| seat rate **at ball contact** (see below) | `0.070000 rad/s` | `0.070000 rad/s` | **unchanged** | anything below `0.0665` (−5 %) — the supersede burst is being throttled |
| reload catch success | as flown (sitting 4: 15/19) | **no worse** | — | a drop attributable to the seat |

**The row that needs its own paragraph: the seat rate AT CONTACT.** Every other
row above is measured on the *pre-tilt* install, whose arrival is `landing −
1.5 s`. That is **not** the plan that runs through the catch. The coordinator
re-asserts the pre-tilt every balls tick, the reach freeze releases at `arrival +
0.5 s` and re-latches at `arrival − 0.3 s`, so **9–11 further catch installs** are
accepted in the last ~0.7 s (reference bag: `landing−0.78..−0.31 s` and
`landing−0.83..−0.29 s`) and the *last* of them is frozen through contact. Each is
seeded already on the target, so its residual travel is ~0.04° against a 10.87°
seat — the regime where an incorrectly-scaled bound silently de-rates the seat
**15.7×** while every row above still reads PASS. The probe now prints

```
!! NOT SCORED BY THIS ROW: N further catch install(s) accepted AFTER the scored arrival,
   landing-0.832..-0.292 s (reach freeze released at landing-1.000 s).
```

- **PASS**: that line appears with `N` between 6 and 25 on a reload attempt, and
  the commanded `ry` in the last 0.8 s before landing tracks the same
  `≈0.9°` round trip the pre-fix session recorded (this is status-quo behaviour
  and is deliberately unchanged by Phase 2).
- **ABORT**: the commanded tilt in the last 0.8 s is **flat** (< 0.1° of motion)
  where the pre-fix capture showed `≈0.9°` — that is the throttled-seat signature,
  and it means the bound's scale regressed to the residual travel. Route back to
  `plans/parked/catch-reach-degenerate-overshoot.md` Phase 2 /
  `ros_ws/docs/catch_arrival_contract.md` § "Why the scale is a MAX".
- Not an abort, but worth logging: `N == 0` means the open-loop republish path did
  not run (check `JB_OP_RELOAD_PLATFORM_OPEN_LOOP` and that the announcement was
  seen) — the sitting is then not exercising this regime at all.

**These are the numbers to watch for at the bench rather than discover there.**
A delta an order of magnitude larger than the table means the receive tilt being
threaded through is not the wire orientation.

### CHECK CCATCH-4 — no motion change anywhere else (regression guard)

Validates: the blast radius really is the catch path.

- `go_to_pose`, `go_home`, `timed_target` and the SpaceMouse follower are
  untouched by C-CATCH-1 (`build_move` / `build_timed` / `build_follow` all take a
  caller-supplied arrival twist and none manufacture one).
- **PASS**: a `go_home` and a `go_to_pose` after the reload goal behave exactly as
  in § Section LVL's CHECK LVL-2 — same durations, same smoothness, no new
  `last_rejection` on `/trajectory/status`.
- **ABORT**: any new rejection code, or a duration change > 5 %.

### CHECK CCATCH-5 — `peak_leg_*` reads 0.0 after a report-less install

Validates: the separate diagnostics fix committed alongside C-CATCH-1
(`tests/hardware/mvp_bench_runbook.md` open item 7).

Six install paths carry no `FeasibilityReport` (`hold`, `go_home`, the guard
descent, the pending-stop retry, the graceful stop, the follower's input-loss
stop). They used to leave `peak_leg_*` describing the **superseded** plan under an
already-advanced `move_seq`. `_install` now clears the field, so those installs
publish `0.0` — "this install carried no prediction".

```bash
# read-only; no actuation
ros2 topic echo /trajectory/diagnostics --once
```

1. Do a `go_to_pose` → note `peak_leg_vel_mmps` (non-zero) and `move_seq`.
2. Call `trajectory/hold` (or `go_home`) → re-read.

| | PASS | ABORT |
|---|---|---|
| after the `go_to_pose` | `peak_leg_vel_mmps > 0`, matching the accepted move | `0.0` — the write is being erased; the ordering regressed |
| after the report-less install | `move_seq` advanced **and** `peak_leg_*` all `0.0` | the previous move's non-zero peaks under the new `move_seq` — the clear did not land (relaunch skipped?) |
| `realized_peak_leg_*` | reset per install, as before | unchanged across an install |

### If CCATCH-2 fails

In this order, cheapest first:

1. Was the build actually installed? `colcon build --packages-select
   jugglebot_interfaces jugglebot` **and** a relaunch — § Build gate. A stale installed copy reproduces the
   pre-fix behaviour exactly and is by far the most likely cause.
2. Run CCATCH-1. A drifted mirror means the probe is scoring old physics.
3. Check the wire: `ros2 topic echo /catch/dynamic_target --once`. If
   `target_quat` is **not** identity for a vertical self-toss, the coordinator is
   sending a real receive tilt and the seat is *supposed* to engage — that is not
   a C-CATCH-1 failure, it is a `compute_catch_orientation` question.
4. Only then suspect the fix. Capture the bag and score it offline; the
   counterfactual block says what the planner would have produced.

---

## Section LVLGATE — `levelling-frame-contract` Phase 3 (`REJECTED_NOT_LEVELLED`)

**Plan**: `plans/parked/levelling-frame-contract.md` § Phase 3
**Contract**: `ros_ws/docs/levelling_frame.md` (**C-LEVEL-1.O**, the
observability half)
**Supersedes two forward references in § Section LVL**: CHECK LVL-1 says
*"`levelling-frame-contract` Phase 3 owns the closure"* — it does; this is it —
and that section's *Not in this section* bullet said the gate had not landed
(corrected in place, since leaving it would tell the operator to ignore a check
that now fires). CHECK LVL-1 itself stays exactly as written: it is still the
right first check, because it reads the two log lines that say a correction was
published **and** received.

**What landed**: the correction is per-PROCESS. `/gravity_offset` is VOLATILE
with one latched push per orchestrator boot and no re-request path, so a
`trajectory_node` that restarts after a `level` holds the identity while
`RobotState.levelling_complete` — a **Teensy-persisted per-boot flag** — still
reads `true`. `trajectory_node` now publishes its own answer on
`trajectory/status.gravity_correction_loaded`, and the toss's CHECKING phase
refuses with `REJECTED_NOT_LEVELLED` unless that answer is **True on a status
less than 1.0 s old**.

Why refuse at all — geometry, not process: un-levelled the launch leaves the cup
0.78° off gravity, and a vertical toss drifts `v·sin θ·T`, which with
`T = 2v/g` and `v = sqrt(2gh)` is exactly **`4·h·sin θ`** — linear in apex
height, and independent of everything else. At the config default apex
(`throw_height_m: 0`, ~0.79 m ⇒ `v = 3.93 m/s`, `T = 0.80 s`) that is **43 mm**
against a **~35 mm** cup radius: the catch is *geometrically impossible* before
the ball leaves the hand.

**The LG commands below use `throw_height_m: 0.6`, where the same formula gives
33 mm** — nominally *inside* 35 mm, and worth stating plainly so nobody
"discovers" it later and concludes the gate over-refuses. It does not, for two
reasons. The cup's usable tolerance is the 35 mm radius **less the ball radius**
(a ball only stays in if its *centre* lands well inside the rim), so 33 mm is
already outside it; and the gate is deliberately **height-independent** — it is
a statement about whether the machine knows where gravity is, not a per-goal
drift budget. Refusing at 0.6 m is the conservative side of a check whose whole
job is to be loud before the ball is airborne. Same class of loud-early reject
as `REJECTED_HAND_NOT_PARKED`.

> ### ⚠ BUILD NEEDS — this section is the ONE with an interface change
>
> `TrajectoryStatus.msg` gained a field, so a `jugglebot`-only build is
> **not enough** (since 2026-07-29 the § Build gate itself specifies both
> packages, so following it is sufficient here):
>
> ```bash
> cd ~/Desktop/Jugglebot/ros_ws
> colcon build --packages-select jugglebot_interfaces jugglebot
> source install/setup.bash
> ```
>
> then **relaunch** `jugglebot_launch.py`. No firmware flash, no config
> regeneration. `/trajectory/status` must be in the bag before LG-1 or LG-4's
> diagnostic cannot be run at the end of the sitting — `record:=true` carries it
> (shared list since 2026-07-26, the ONE list since 2026-08-10).
>
> **If you rebuild `jugglebot` but not `jugglebot_interfaces`,
> `trajectory_node` DIES — it does not merely go quiet.** `_publish_status`
> assigns the new field to a generated message whose `__slots__` lack it,
> raising `AttributeError` inside the 0.2 s timer callback; rclpy re-raises
> timer exceptions out of `spin()` (`rclpy/executors.py` `spin_once`) and
> `trajectory_node.main` catches only `KeyboardInterrupt`, so the **process
> exits ~200 ms after launch**. What you will actually see is: no
> `trajectory_node` in `ros2 node list`, no 40 Hz hold stream, `ros2 topic echo
> /trajectory/status` hangs, and **`activate` FAILS at the A2 arm** ("no mpccmd
> frame") — you never reach TRAJECTORY, so you never send a toss at all. Do
> **not** go hunting the levelling gate on that signature. LG-0 catches it in
> three seconds.

> ### ⚠ OPERATOR PRE-BRIEF
>
> 1. **You will now get a loud refusal instead of a wasted throw if you forget
>    to `level`.** ~~`levelling_complete` is "since the last Teensy bootup" and the
>    § Shared preconditions power-cycle the can-bridge Teensy every sitting, so
>    it is `false` at every launch and the orchestrator's persisted auto-push
>    never fires first. **In practice every session genuinely needs a manual
>    `level`.**~~
>    *(Amended 2026-08-15, **resolved 2026-08-16 from source**. Both halves of the
>    struck reasoning are dead. The § Shared preconditions power-cycle is
>    **retired** with standing rule 1, so it no longer happens every sitting; and
>    it was the wrong board anyway — `levelling_complete`/`pose_offset_rad` live in
>    a RAM global on the **Platform** Teensy, which is on Jugglebot's 12 V/ODrive
>    supply, while the can-bridge is on the Jetson's 5 V rail and holds no copy, so
>    a can-bridge power-cycle **cannot** clear them. Standing rule 2 and item 2
>    below were right; this item was wrong. Full derivation:
>    `ros_ws/docs/levelling_frame.md` § "Which board owns `levelling_complete`".
>    The conclusion to act on is item 2: **read `gravity_correction_loaded`, never
>    assume it** — a within-sitting relaunch normally comes back already levelled.)*
> 2. **After a relaunch, CHECK `gravity_correction_loaded` before re-levelling.**
>    *(Corrected 2026-07-27 — this item previously said to re-`level` after every
>    relaunch.)* The correction does live in `trajectory_node`'s memory, but the
>    orchestrator re-pushes the **Platform** Teensy's persisted offset on the first
>    IDLE entry after boot, and standing rule 1's power-cycle is the *can-bridge*
>    board, which does not clear that cache. `/gravity_offset` is VOLATILE, so the
>    push can lose a discovery race — which is exactly why you read the flag rather
>    than assume either way. See standing rule 2.
> 3. **A zero offset still counts as levelled.** A genuinely level machine
>    measures ~0 tilt; the gate asks whether a measurement was *taken and
>    pushed*, never whether it was large. If `level` reports ~0 and the toss
>    proceeds, that is correct.
> 4. **`REJECTED_NOT_LEVELLED` has a second, rarer cause**: `trajectory/status`
>    went silent for > 1 s, i.e. **`trajectory_node` is dead or stalled**. It
>    reports as a levelling refusal rather than as `REJECTED_NOT_STREAMING`
>    because `streaming` is a sticky last-value with no freshness stamp of its
>    own (deliberately — the reload FSM shares that field; see the plan's
>    Phase 3 Outcome, *Deliberately NOT done*). So: **if a fresh `level` does
>    not clear the refusal, stop looking at the levelling routine** and run
>
>    ```bash
>    ros2 node list | grep trajectory_node          # decisive: present or not
>    ros2 topic echo /trajectory/status --once      # returns promptly, or hangs
>    ```
>
>    (`ros2 topic hz` is unreliable on this Foxy box for RELIABLE topics — use
>    the two above.) A missing node or a hanging echo means the fault is the node, not the
>    frame — LG-0 and the LG-4 fallback below. (This is not the
>    stale-interface case: that one kills the node before you can arm, so you
>    never get as far as a toss.)

### Pre-flight LG-0 — confirm the freshly-built interface and code are live

Read-only, no robot, ~3 s. Run this **before** anything else in this section.

```bash
grep -c gravity_correction_loaded \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot_interfaces/share/jugglebot_interfaces/msg/TrajectoryStatus.msg
grep -c gravity_correction_loaded \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/trajectory_node.py
grep -c "NOT_LEVELLED" \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/toss_sequencer.py
```

- **PASS**: all three counts are **non-zero** (at the Phase-3 commit they read
  `1`, `3`, `2` — treat the exact numbers as informational, since a later
  comment edit moves them; **zero** is the failure).
- **ABORT**: any count is `0` — the *installed* copy predates Phase 3. Rebuild
  **both** packages per the build box above and relaunch. Scoring LG-1..LG-5
  against a stale install proves nothing.

With the graph up, confirm the node is alive and the field is actually on the
wire:

```bash
ros2 node list | grep trajectory_node
ros2 topic echo /trajectory/status --once | grep -E "streaming|gravity_correction_loaded"
```

- **PASS**: `trajectory_node` is listed, and both keys print. Before any
  `level`, expect `gravity_correction_loaded: false`.
- **ABORT**: `trajectory_node` is **absent** — that is the half-rebuild
  signature (it raised on its first status tick and the process exited). Rebuild
  both packages and relaunch.
- **ABORT**: the node is present but the key is missing, or the echo hangs —
  a *different* publisher build, or the node is not publishing. Fix before
  continuing.

### CHECK LG-1 — a toss BEFORE `level` is refused (**the headline check**)

Validates: Phase 3, the gate itself.

**Do this with an EMPTY cup.** If the gate fails to fire, the sequence proceeds
to a dry toss — an empty-hand stroke, the same operation
`session_phase8_toss_trace.md` runs deliberately — which is safe but is not
something to discover with a ball in the hand. **Do not run LG-1 with a ball
seated.**

Sequence: launch → home (if needed) → activate → TRAJECTORY (streaming) →
**do NOT `level`** →

```bash
ros2 topic echo /robot_state --once | grep levelling_complete
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```

- **PASS**: `outcome: REJECTED_NOT_LEVELLED`, the goal terminates within
  **< 1.0 s** of accept (a CHECKING reject fires on the first FSM tick, ~50 ms —
  anything near the 6 s positioning timeout means a *different* gate fired
  late), the platform does **not** move, and the hand does **not** move.
- **PASS (variant, record it)**: `levelling_complete: true` and the toss
  proceeds. Then a persisted correction was still aboard, the orchestrator's
  auto-push fired at the first IDLE, and a correction is genuinely loaded — the
  gate is right to pass. Do not score this as a failure, and do not score it as a
  pass of the gate either. **To force the un-levelled precondition, power-cycle
  the PLATFORM Teensy** — its `RobotState` is a RAM global zero-initialised at
  boot, so the cycle drives `levelling_complete` false — and re-run LG-1. **Budget
  a re-home**: that board is on Jugglebot's 12 V/ODrive supply, so the same power
  event clears `is_homed` and the ODrive references too.
  *(Amended 2026-08-15, confirmed from source 2026-08-16: this step used to say
  "power-cycle the Teensy per § Shared preconditions", i.e. the can-bridge board —
  which stores no copy of the flag and cannot clear it. The can-bridge power-cycle
  precondition is itself now retired. Derivation:
  `ros_ws/docs/levelling_frame.md` § "Which board owns `levelling_complete`".)*
- **ABORT**: the toss proceeds with `levelling_complete: false`. The gate is not
  live — almost certainly LG-0 (stale install). Stop; nothing else in this
  section is meaningful.
- **ABORT**: any platform or hand motion during a `REJECTED_*` goal. A CHECKING
  reject runs `ACTION_NONE` — nothing has moved or armed yet — so motion here is
  a different fault entirely.

### CHECK LG-2 — after `level`, the same goal is NOT refused

Validates: the negative half. A gate that refuses a correctly-levelled machine
gets bypassed, which is worse than no gate.

**Write the state transitions out, because `level` is only accepted from
IDLE.** LG-1 leaves the robot in ACTIVE:TRAJECTORY, where `level` is *silently
discarded* (`state_machine.IdleHandler` is the only handler that consumes it),
and `LevellingHandler` ends at `level_deactivate` and returns to **IDLE** — so
you must re-activate afterwards. The full sequence, mirroring § Section LVL
CHECK LVL-2:

**`deactivate` → `level` → `activate` → `trajectory` → re-issue the goal.**

Confirm CHECK LVL-1's two log lines after the `level`, then re-issue the
**same** command as LG-1 (still empty cup — it will dry-toss, which is fine; or
load a ball and let this be the § Section LVL CHECK LVL-3 toss, one capture
scoring both).

```bash
ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded
```

- **Not a verdict, fix the state**: `REJECTED_WRONG_MODE` means you are not in
  ACTIVE:TRAJECTORY — re-activate and re-enter the mode, then re-issue. Nothing
  about the levelling gate has been tested yet.

- **PASS**: `gravity_correction_loaded: true`, and the toss's feedback advances
  past `CHECKING` to `POSITIONING`. The outcome from there on is § Section LVL's
  business, not this section's.
- **PASS (equivalent)**: `level` reported an offset of ~0 and the toss still
  advances. Correct — see pre-brief item 3. Record that this case applied.
- **ABORT**: `gravity_correction_loaded: false` immediately after a `level`
  whose two log lines both appeared. That is a genuine Phase-3 defect (the flag
  is not being set where the correction is stored) — capture
  `~/.ros/log` and stop.
- **ABORT**: `REJECTED_NOT_LEVELLED` after a clean `level`. Re-read
  `gravity_correction_loaded` a second time before concluding: if it is `true`
  but the toss still refuses, the coordinator is not consuming it (or its status
  is stale — LG-4).

### CHECK LG-3 — the relaunch case: the Teensy flag lies, the gate does not

Validates: **the design decision this phase turns on.** This is the check that
distinguishes the shipped gate from the one the plan originally specified.

> ### Reachability — SETTLED 2026-08-16, from source
>
> `logbook/2026-07-28-anomaly-fixes-validation-sitting.md` § "`LG-3` — an
> unresolved disagreement" recorded two analysts disagreeing about whether this
> check's precondition (`levelling_complete: true` **and**
> `gravity_correction_loaded: false`) is reachable at all. It is — but **by
> neither route either of them proposed.**
>
> - **The Platform-Teensy power-cycle recipe is REFUTED.** That board's
>   `RobotState` is a RAM global zero-initialised at boot
>   (`Teensy_code_platform.ino:139-145`; no EEPROM in the sketch), so the cycle
>   drives `levelling_complete` **false** — which is LG-1's state, not this one's.
> - **"Unreachable by any power-cycle" is CORRECT**, for the same reason plus its
>   mirror: a can-bridge cycle clears nothing (that board stores no copy), so it
>   leaves the flag true *and* the orchestrator free to auto-push.
> - **"So the honest closure is a unit test" is TOO STRONG.** The state is
>   reachable on hardware, deterministically, with no power-cycle at all. The
>   orchestrator's persisted push is **one-shot per orchestrator boot**
>   (`orchestrator_node.py:130` `_startup_offset_sent = False`, set True at
>   `:328-334` on the first IDLE entry and never reset), `/gravity_offset` is
>   VOLATILE with no re-request path, and `trajectory_node` starts with
>   `_gravity_correction_loaded = False` (`trajectory_node.py:362`). So: `level`,
>   then restart **`trajectory_node` alone**, leaving the orchestrator up. The
>   Platform Teensy still holds `levelling_complete = true`; the orchestrator will
>   not re-push (its latch is spent); the new `trajectory_node` holds identity.
>   No race.
>
> **Run it the alone-restart way** (below); it is the only route that establishes
> the precondition by construction. **Bench-verify once before trusting it**: the
> mechanics of restarting that one node under the launch are un-exercised —
> `jugglebot_launch.py` sets no `respawn=`, so the killed node stays dead and
> `ros2 run jugglebot trajectory_node` restarts it, but `trajectory_node` is the
> **sole binder of :5557** and the replacement must be able to re-bind after the
> old process exits. If the re-bind fails, fall back to the whole-graph relaunch
> below and score it only if the flags come up right.

**Route A (deterministic — preferred).** From the LG-2 state (levelled, toss
accepted), **without running `level` again**:

```bash
pkill -f 'trajectory_node'                     # orchestrator stays up
ros2 run jugglebot trajectory_node             # new process, empty correction
```

then re-arm if `control_mode` was disturbed and read both flags below.

**Route B (opportunistic — the original).** **Relaunch `jugglebot_launch.py`
without re-levelling.** A relaunch blanks `control_mode`, so you must re-arm
before the goal means anything:

**relaunch → `activate` → `trajectory` → read both flags → send the goal.**

This route reaches the precondition **only if the boot auto-push loses the
discovery race** — on 2026-07-27 the push won all 7 publishes, the boot-push
subset 5/5 — so read the flags first and score nothing if
`gravity_correction_loaded` is already `true`.

Then, empty cup:

```bash
ros2 topic echo /robot_state --once | grep levelling_complete
ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```

- **PASS**: `levelling_complete: true` **and** `gravity_correction_loaded: false`
  **and** `outcome: REJECTED_NOT_LEVELLED`. Those three lines together are the
  whole point: the Teensy still says "this machine has been levelled", the node
  that actually applies the correction says it holds none, and the gate believes
  the second one.
- **Not a verdict, fix the state**: `REJECTED_WRONG_MODE` means you skipped the
  re-arm above (post-relaunch `control_mode` is empty, and that gate fires
  *before* CHECKING). Re-activate, re-enter TRAJECTORY, re-issue — **without**
  running `level`, or the check is spoiled.
- **ABORT**: `levelling_complete: true` and the toss **proceeds**. The gate is
  wired to the Teensy flag — the exact false assurance this phase exists to
  remove. Stop and report; a passing gate here is worse than no gate, because it
  would launch a 0.78°-off throw with an all-clear.
- Then `level` again and confirm the refusal clears (this is LG-2 repeated, and
  it is also the standing operational requirement in pre-brief items 1–2).

### CHECK LG-4 — no spurious refusals across the sitting

Validates: the 1.0 s freshness window on `trajectory/status` is not tight enough
to mint false refusals on a healthy machine.

At the **end** of the sitting, over every toss goal issued **after** the last
`level`:

```bash
ls -t ~/.ros/log/python3_*.log | head -20 | xargs grep -h "Toss REJECTED_NOT_LEVELLED" | wc -l
```

- **PASS**: `0`.
- **REPORT, do not abort**: a count equal to the number of goals you deliberately
  issued before a `level` (LG-1, LG-3) — subtract those; they are the successes.
- **ABORT**: any refusal on a goal issued after a `level`, with
  `gravity_correction_loaded: true` at the time. That is a staleness false
  positive.

If that last case happens, measure the gap that caused it before changing
anything — the window was sized against **measured** inter-arrivals (median
200.0 ms, p99 210 ms, **max 508.5 ms** over 420 s of the two 2026-07-25 reference
bags), so a breach is new information about the machine, not a wrong constant:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python - <<'PY'
from pathlib import Path
from mcap.reader import make_reader
bag = sorted(Path.home().joinpath('Desktop/rosbags').glob('*/*.mcap'))[-1]
ts = []
with open(bag, 'rb') as fh:
    for _s, ch, m in make_reader(fh).iter_messages():
        if ch.topic == '/trajectory/status':
            ts.append(m.log_time / 1e9)
ts.sort()
gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
print(bag.name, 'n=%d' % len(ts),
      'median=%.1fms' % (gaps[len(gaps)//2]*1e3),
      'max=%.1fms' % (gaps[-1]*1e3),
      'over_1s=%d' % sum(g > 1.0 for g in gaps))
PY
```

- `/trajectory/status` must have been in the bag from the **start** of the
  sitting for this to work — `record:=true` carries it (§ Recording). This
  measurement cannot be reconstructed afterwards, which is exactly why the two
  divergent record lists were collapsed into one on 2026-08-10.
- `over_1s > 0` ⇒ the window is genuinely too tight for this machine's load;
  raise `_TRAJ_STATUS_STALE_S` with the measured max in the commit message.
  `over_1s == 0` ⇒ the refusal came from somewhere else; do not touch the
  constant.

### CHECK LG-5 — the cross-process ordering (**the one unit tests cannot reach**)

Validates: the wiring `orchestrator_node → /gravity_offset → trajectory_node →
/trajectory/status → reload_coordinator_node → CHECKING`. Every test behind this
phase runs against mocked ROS, which is blind to cross-process message
choreography by construction — two of them feed a real `trajectory_node`'s own
`_publish_status` output into the real coordinator callback, but nothing in the
suite can prove that the two *processes* actually exchange it in that order on
this box. **This check is the only thing that does.** It costs nothing extra: the same recorder capture that
§ Section HAND and CHECK LVL-3 already require.

Run `tests/hardware/toss_trace_recorder.py record` (see § Section HAND for the
invocation) spanning: LG-1's refused goal → `level` → LG-2's accepted goal. Then,
against the resulting `temp/logs/toss_trace_<stamp>.jsonl`:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python - "$(ls -t temp/logs/toss_trace_*.jsonl | head -1)" <<'PY'
import json, re, sys
# Same outcome-line definition the recorder's own checker uses (OUTCOME_RE +
# the reload_coordinator_node filter), so this reader and `check` agree about
# what counts as an outcome. A bare startswith('Toss ') would also catch the
# coordinator's 'Toss early exit while prepared ...' and 'Toss displaced aim
# infeasible ...' lines and mis-number "the first outcome".
OUTCOME_RE = re.compile(r'^Toss ([A-Z][A-Za-z0-9_()]*)')
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
st = [(r['t'], r['d'].get('gravity_correction_loaded'))
      for r in rows if r.get('topic') == 'trajectory/status']
flips = [(t, v) for i, (t, v) in enumerate(st)
         if i == 0 or v != st[i-1][1]]
outc = []
for r in rows:
    if r.get('topic') != '/rosout':
        continue
    if r['d'].get('node') != 'reload_coordinator_node':
        continue
    m = OUTCOME_RE.match(str(r['d'].get('msg', '')))
    if m:
        outc.append((r['t'], m.group(1)))
print('status rows: %d   loaded-flips: %s' % (len(st), flips))
for t, oc in outc:
    print('  t=%.3f  %s' % (t, oc))
if len(flips) >= 2 and len(outc) >= 2:
    print('1st outcome -> flip : %+.3f s  (want POSITIVE)' % (flips[1][0] - outc[0][0]))
    print('flip -> 2nd outcome : %+.3f s  (want > +0.005)' % (outc[1][0] - flips[1][0]))
else:
    print('INCOMPLETE: need >=2 flips and >=2 outcomes; see the ABORT rows')
PY
```

- **PASS**, all four, and the two printed separations are the numeric test of
  (2) and (3) — do not eyeball the timestamps:
  1. `loaded-flips` has exactly **two** entries and the first is `(…, False)`
     (the pre-`level` state);
  2. the first outcome is `REJECTED_NOT_LEVELLED`, and
     `1st outcome -> flip` is **positive** (the refusal precedes the flip);
  3. the second flip entry is `True`, and `flip -> 2nd outcome` is
     **> +0.005 s**;
  4. the second outcome is **not** `REJECTED_NOT_LEVELLED`.
- **AMBIGUOUS, not PASS**: `flip -> 2nd outcome` is between `0` and `+0.005 s`
  (the recorder's `--epsilon-ms` honesty rule — rows are stamped at callback
  execution, and two messages in one executor wait-set cycle can be observed in
  either order). Re-run; a real capture separates them by the 5 Hz status
  period, ~0.200 s. A **negative** value is not ambiguity — the second goal ran
  before the correction landed, so re-run with the `level` completed first.
- **ABORT**: the flip never happens although CHECK LVL-1's `gravity correction
  set` line did — the field is not reaching the wire, and every LG check above
  was scoring the freshness half only.
- **ABORT**: `loaded-flips` shows `True` before the `level`, with the **Platform**
  Teensy power-cycled. Something republished a stale correction; the gate is then
  reporting a frame nobody established this session. *(Amended 2026-08-15: this
  row used to say "the Teensy power-cycled per § Shared preconditions", which is
  now a dangling reference twice over — standing rule 1's can-bridge power-cycle
  is retired, and it was the wrong board anyway: the levelling cache lives on the
  Platform Teensy, per standing rule 2 and the amendment at CHECK LVL-2's setup.)*
- **ABORT**: `INCOMPLETE` — fewer than two flips or fewer than two outcomes.
  One of the two ABORT rows above applies; read `loaded-flips` to see which.

> **This reader was validated in both directions before it shipped**
> (2026-07-26, finalize): four synthetic captures in the recorder's own JSONL
> schema — the clean `False → REJECTED_NOT_LEVELLED → flip → MISSED` shape it
> must ACCEPT (all four criteria readable, separations `+7.200 s` / `+12.800 s`),
> plus never-flips, `True`-before-`level`, and a 2 ms flip/goal separation, each
> of which it must FLAG (and does). Two decoy `/rosout` lines — the
> coordinator's own `Toss early exit while prepared …` and a `Toss …` line from
> another node — are correctly excluded by the `OUTCOME_RE` + node filter. An
> instrument validated only against the broken shape scores a working fix as a
> failure and burns the sitting.

### Not in this section

- Whether the platform is actually level, and by how much — § Section LVL
  (CHECK LVL-1..LVL-4). This section only asks whether the machine *knows*.
- The pre-throw tilt swing, the catch error — § Section LVL and § Section CCATCH.
- Scoring this capture with `toss_trace_recorder.py check --reject`: that path
  expects exactly one `REJECTED_NO_BALL` and will FAIL on a `NOT_LEVELLED`
  trace. Use the inline reader above; the `check` subcommand is for the dry and
  no-ball traces, unchanged.

---

## Section ZSEAT — the manufactured through-seat rate ships at ZERO

> **Appended 2026-07-26.** This section is an **operator decision**, not a bug
> fix, and it lands after § Section CCATCH.
> `planner._CATCH_TILT_THROUGH_RATE_RADPS` is now **`0.0`**.
>
> It supersedes **five** rows of **§ CHECK CCATCH-3** (and leaves a sixth, the
> `arrival-rate bound`, *stale* rather than false) — a banner at the head of that
> check lists them, and one of them (*flat commanded tilt in the last 0.8 s*) is
> **inverted**: what CCATCH-3 tells you to ABORT on is now the PASS. The fifth,
> added 2026-07-27, is `peak off the park`, which moves `−0.292407°` on a HEALTHY
> capture against its own `> 0.10°` ABORT. Read that banner before scoring any
> reload.
>
> Nothing in **§ CHECK CCATCH-2** changes. A gravity-level catch already had a
> zero arrival twist under C-CATCH-1 (`smag == 0`), so every CCATCH-2 number is
> untouched by this change.

**What changed, and what deliberately did not.** The trajectory builder no longer
**manufactures** an arrival tilt rate. Every catch nobody gave an opinion about is
now *reach + quiescent hold*: two segments, the rim stationary at ball contact,
the settle exactly on the commanded target, and the plan 0.15 s shorter.

What did **not** change is the caller seam. Platform motion during a catch or a
throw is **permitted** — an explicitly-supplied `tilt_through_rate_radps` is still
honoured verbatim and unbounded, still builds the decay segment, and is what a
future optimising planner will use. It is simply never **mandated** by a constant
in the builder. There is no rule anywhere that a catch must command no motion, and
none should be added; C-CATCH-1 itself still contains no stationarity clause.

**Prerequisites:** `cd ~/Desktop/Jugglebot/ros_ws && colcon build
--packages-select jugglebot_interfaces jugglebot && source install/setup.bash`,
then **relaunch** (the two-package build is mandatory in EVERY section since 2026-07-29 — `reload_coordinator_node` imports `TossContinuous` at module scope, so a `jugglebot`-only build raises `ImportError` before that node is constructed and takes `Reload`, `Toss` and `TossContinuous` down together; matrix row B) — the
launch runs the *installed* copy, and the change is in
`motion/trajectory/planner.py`. **No firmware flash. No config regeneration.**
Standing session rules still apply — as amended: the can-bridge power-cycle is
**retired** (standing rule 1, 2026-08-15), and you still **check**
`gravity_correction_loaded` after the relaunch — `level` only if it reads
`false` (standing rule 2, corrected 2026-07-27).

### THE RISK THIS SECTION EXISTS TO SCORE — read before the sitting

The `0.07 rad/s` seat rate existed for one physical reason: **a parked *tilted*
rim deflects the ball** (the bb-sim geometry finding). Two catches, two very
different exposures:

- **The self-toss catch seats LEVEL.** A level rim has no deflection geometry, so
  zero costs it nothing — and under C-CATCH-1 its seat rate was already zero.
  **No new risk on this path.**
- **The reload catch seats at 11.08° of tilt.** That is exactly the geometry the
  bb-sim finding concerns, and this change makes that rim **stationary at
  contact**. **This is the live risk, and it is not mitigated in code — on
  purpose.**

Two facts that bound the risk without removing it, both worth knowing before you
score a miss:

1. `0.07 rad/s` has **never been validated on hardware**. It was an estimate, and
   its own leg-velocity sizing note was 2× wrong until it was measured
   (2026-07-26: 14.24 mm/s, not ~7).
2. Until commit `407154f` the seat was aimed off the **plan-frame** tilt, so on
   every levelled catch it pointed along the levelling correction rather than at
   the ball. Any bench impression of the seat formed before that commit was formed
   on a **mis-aimed** seat, and is not evidence that a correctly-aimed one helps.

So the reload is the experiment. Score it, do not argue it.

### CHECK ZSEAT-1 — instrument health (run FIRST, no bag, no robot)

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py --self-check
```

- **PASS**: `SELF-CHECK: PASS`, **10/10** `OK`, exit 0. Case 7 must show
  `planner._CATCH_TILT_THROUGH_RATE_RADPS=0.0` **and**
  `recorded-session rate (capture record, NOT a live mirror)=0.07`.
- **ABORT**: `planner._CATCH_TILT_THROUGH_RATE_RADPS` reads anything but `0.0` —
  the source tree is not the one this section describes. Stop; do not score.
- Note: the *recorded-session* `0.07` is deliberately **not** synced to the new
  default. It is what the reference bag ran at, and `build_replay` passes it
  explicitly so pre-2026-07-26 captures keep reproducing.

### CHECK ZSEAT-2 — reload seating and bounce-out (**the scored row**)

Validates: that removing the seat from an 11.08° tilted rim did not cost catches.
This is the check the whole section exists for, and a failure routes back here —
not to C-CATCH-1, not to the levelling contract.

Run a normal reload sitting, **≥ 12 reload attempts** (sitting 4's 19 is the
reference sample size; below ~12 the binomial noise swamps the effect you are
looking for). Record with `record:=true` — the ONE list (§ Recording) carries
`/trajectory/diagnostics /trajectory/target_feedback /catch/dynamic_target
/gravity_offset /throw_announcements` as of 2026-08-10.

Score every attempt by eye as well as by `outcome` — the tracker still reports
`MISSED` on real catches, so `outcome` alone is not the verdict anywhere in this
file. For each attempt record one line: **caught / bounced-out / missed-arrival**,
where *bounced-out* means the ball **touched the cup and left it**. Bounce-out is
the failure mode this change could newly cause; a missed arrival (ball never
reached the cup) is BB scatter and is not evidence about the seat.

| quantity | PASS | ABORT (route back to this section) |
|---|---|---|
| caught / attempted | **rate ≥ 0.63** (≥ 8/12, ≥ 12/19) — i.e. no worse than sitting 4's 15/19 (0.79) by more than one binomial sigma (σ ≈ 0.09 at n = 19) | **rate ≤ 0.58** (≤ 6/12, ≤ 11/19), i.e. ≥ 2σ down |
| caught / attempted, **between** those two rates | — | **INCONCLUSIVE, not a verdict.** Score the RATE, never a fixed count: the sitting mandates only `n ≥ 12`, where 7/12 = 0.583 satisfies *neither* predicate and a literal reading of "12/19" as *twelve catches* is unachievable at n = 12 at all. Report the raw count and either extend the sitting toward `n = 19` or repeat |
| **bounce-outs** (touched the cup, left it) | **≤ 1** across the sitting | **≥ 3**, or **≥ 2 consecutive** — this is the seat-deflection signature and it is what the `0.07` existed to prevent |
| bounce-out vs sitting 4 | sitting 4's misses were BB scatter + late arrival, **not** bounce-out | any bounce-out cluster that was **absent** before is attributable to this change until shown otherwise |
| commanded tilt over the last 0.8 s before landing (FK of `/leg_setpoint_echo`) — `levelling_tilt_bag_check.py --t0 <landing−0.8> --t1 <landing> --plateau-min-s 0.1 --plateau-tol 1.0` (**seconds from the bag's first `/leg_setpoint_echo` sample** — NOT the ROS epoch and NOT `--list`'s `rel_first_s`; see **§ The `--t0/--t1` origin note**, in THE RUN SHEET above, stage 6), read `span_deg`. **Both flags are required** — at the 0.3 s default a 0.8 s window of real motion yields no plateau and the probe prints only an ERROR, ignore its `VERDICT:` | **flat**, `< 0.05°` of motion — the rim is parked, as intended | `≈ 0.9°` of round trip — the zero default did **not** land (stale install: colcon + relaunch) |
| `peak_leg_acc_mmps2` / `_jerk_mmps3` on a reload catch install | `≈ 38 / ≈ 170` (was `142 / 3935`) | still `≈ 142 / ≈ 3935` — stale install |
| plan segments on a reload catch install | **2** | 3 — stale install |

**If ZSEAT-2 ABORTs on bounce-out**, the finding is *"a stationary tilted rim
deflects the ball on hardware, and the bb-sim geometry finding is confirmed"*.
That is a real, publishable result and the fix is a **one-line default**, not a
redesign: raise the seat rate off zero and re-run. **Raise it in the YAML, never
in `planner.py`** — it is `trajectory_op.catch_seat_rate_radps` in
`config/hardware_config.yaml` since 2026-07-28, and moving it needs all four of
YAML → `generate_config.py` → `colcon build` → relaunch (§ DEPLOYMENT MATRIX row
D). A source edit at the bench is an ABORT under `SEAT-EXP-8.3`: it leaves no
trace in the *installed* tree, so the capture cannot be attributed to a rate, and
it can be forgotten and shipped as an accidental permanent default. **Do not
improvise the A/B from this paragraph — it has a pre-registered protocol:
§ SECTION SEAT-EXP**, which fixes the warm-up discipline, the matched-arrival
requirement and the decision rule *before* the data exists.
C-CATCH-1 is still in force and will bound whatever value goes in — the contract
was kept live for exactly this moment. Log the bounce-out count and the value
tried; that is the start of the seat-tuning session the constant's docstring has
always anticipated.

### CHECK ZSEAT-3 — the offline counterfactual agrees with the machine

Validates: that the capture matches the numbers this change was reasoned from.
Read-only, on the ZSEAT-2 bag.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> --list
python tools/probes/catch_reach_replay.py --bag ~/Desktop/rosbags/<SESSION> \
    --thrower ball_butler --toss N --json
```

The **C-CATCH-1 COUNTERFACTUAL** block prints `MANUFACTURES NOTHING` on its
`arrival-rate bound` line whenever the live planner default is `0.0`. **Match on
that fragment only** — the surrounding wording changed on 2026-07-28 (the line
now names the *live* planner default rather than a mirrored constant, and appends
a `[!]` drift callout when the tree is off its shipped seat rate), and it will
change again if the seat experiment moves the default. A wording mismatch here is
NOT the stale-install signal that `SEAT-EXP-1`/`3` gate on; those rows read the
installed constant and the self-check exit code, which are exact. On a *post-fix*
capture the recorded reach and the counterfactual should collapse together.

Reference deltas, measured 2026-07-26 offline on
`~/Desktop/rosbags/2026-07-25_15-17-48 --thrower ball_butler --toss 2`
(lead 2.3712 s, wire receive tilt 10.87°) — **C-CATCH-1 at `0.07` → shipped
`0.0`**:

| quantity | rate 0.07 | **rate 0.0 (shipped)** | delta | ABORT if |
|---|---|---|---|---|
| arrival tilt rate at contact | `0.070000 rad/s` (4.011 °/s) | **`0.000000`** | −0.070000 | non-zero — stale install |
| settle `rx` | `+1.844635°` | **`+1.774062°`** (= target) | `−0.070573°` | \|settle − target\| > 0.02° |
| settle `ry` | `−10.928741°` | **`−10.636334°`** (= target) | `+0.292407°` | \|settle − target\| > 0.02° |
| residual past the seat | `0.300803°` | **`0.000000°`** | −0.3008° | > 0.02° |
| segments | 3 | **2** | −1 | 3 |
| plan duration | lead + 0.65 s | **lead + 0.50 s** | −0.15 s | lead + 0.65 s |
| predicted `peak_leg_vel_mmps` | `23.8` | **`29.0`** | **+22 %** | > 60 (session ceiling 1000) |
| predicted `peak_leg_acc_mmps2` | `142.0` | **`37.9`** | −73 % | > 200 (session ceiling 5000) |
| predicted `peak_leg_jerk_mmps3` | `3935` | **`170`** | −96 % | > 500 (session ceiling 30000) |
| seat aim rotation (unchanged by this change) | `4.0997°` | `4.0997°` | — | > 6° |

**The velocity row is not a typo and is not a regression.** A terminal tilt rate
pointing along the travel lets the reach coast slower through its middle
(`p' = d·ψ'/T + v1·φ'`, and `φ'(0.5) = −0.4375`), so removing it restores the
plain rest-to-rest `1.875·d/T` peak. 29 mm/s against a 1000 mm/s ceiling. Stated
here because *"every number got smaller"* would be a false expectation, and an
operator who held it would ABORT on a healthy capture.

### CHECK ZSEAT-4 — the throw is stationary at release, on both tiers

Validates: the operator asked for stationary catches **and throws**. This confirms
there is no residual *commanded* platform rate at the release instant. It is a
read of the capture, not a new manoeuvre.

Reason it is expected to hold (verified against the code 2026-07-26, and by
timing rather than by inspection of a trace alone):

- **Tier 8a** runs the stock announcement pre-tilt, i.e. a `build_catch` plan with
  arrival at `landing − 1.5 s` and a 0.5 s quiescent hold, then `hold_after=True`
  freezes the final pose. Release is at `landing − flight`, and the shipped flight
  band is `0.55 … 1.10 s`, so release always falls in the **zero-twist hold** (long
  flights) or after the plan has ended (short flights). A release inside the reach
  would need a flight > 1.5 s, outside the band.
- **Tier 8b** suppresses that pre-tilt (`catch/pretilt_hold`) and holds the
  positioned pose at A open-loop; the deferred A→B reach is published **at**
  `t_release`, never before. At the release instant the platform is in
  `go_to_pose`'s terminal hold.

| quantity | PASS | ABORT |
|---|---|---|
| commanded platform pose (FK of `/leg_setpoint_echo`) over `release ± 0.10 s` — produce it with `levelling_tilt_bag_check.py --t0 <release−0.10> --t1 <release+0.10> --plateau-min-s 0.05 --plateau-tol 1.0` (**seconds from the bag's first `/leg_setpoint_echo` sample** — NOT the ROS epoch and NOT `--list`'s `rel_first_s`; see **§ The `--t0/--t1` origin note**, in THE RUN SHEET above, stage 6) and read the plateau table's `span_deg` for the tilt, and the `commanded position span (x,y,z) mm` line for the mm figure. **Both flags are required**: the default `--plateau-min-s` is 0.3 s, so a 0.2 s window yields no plateau at all, and the probe then prints only `ERROR: no rx plateau …` and returns BEFORE the sample count, the plateau table and the position-span line — the healthy and unhealthy cases would print the same thing; ignore that probe's `VERDICT:` line, which scores the *park* against the offset and means nothing on a 0.2 s window | **flat**, `< 0.02°` and `< 0.2 mm` of motion | any commanded motion — a plan is running through the release |
| `/trajectory/status` `plan_time_remaining_s` at release | ≤ 0 (terminal hold), or a hold segment | a `move` plan mid-reach |
| Tier 8b only: first `catch/dynamic_target` timestamp | **≥** `t_release` | before `t_release` — the deferred reach fired early |

### Not in this section

- Whether the *level* (self-toss) catch commands a swing — § CHECK CCATCH-2,
  unchanged by this section.
- Whether the seat's **aim** is right — § CHECK CCATCH-3's aim rotation row, still
  valid.
- Re-tuning the seat rate. That is the follow-on session ZSEAT-2's ABORT path
  opens, and it now HAS its protocol in this file: **§ SECTION SEAT-EXP**
  (added 2026-07-28, a separate later sitting — do not fold it into this one).

---

## SECTION TIER — Tier 8b is the shipped default

> **AMENDED 2026-08-10 — the default went BACK to `8a`.** `toss_tier` was flipped
> `"8b" → "8a"` on 2026-08-10 (owner decision after the tilt-map validation and an
> 8a toss retest; see
> [`logbook/2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md`](../../logbook/2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md)
> § *Arc wrap-up*). This whole section — including **`CHECK TIER-PREREQ`, which
> ABORTS on `'8a'`** — describes the 2026-07-28 → 2026-08-10 window: on a default
> build today TIER-PREREQ reads `JB_OP_TOSS_TIER = '8a'`, which is now **correct,
> not a failed build**. To run this section as written you must flip to `"8b"`
> first (`config/hardware_config.yaml` → `python config/generate_config.py` →
> `colcon build --packages-select jugglebot` → relaunch) and flip back afterwards.

> **Appended 2026-07-28.** This section is an **operator decision**, not a bug fix,
> and it lands after § Section ZSEAT. `jugglebot_operational.toss_tier` is now
> **`"8b"`** (was `"8a"`). Validates
> `plans/parked/catch-reach-degenerate-overshoot.md` **Phase 4** and
> `plans/active/single-ball-toss.md` **Phase 4**; a failure routes to whichever of
> the two the failing row names.
>
> It changes **no criterion in any section above**. Every row above nominates a
> catch at `(0, 0, 170)`, i.e. `|B − A| = 0`, and at zero displacement the Tier-8b
> release state reproduces Tier 8a **bitwise** (`toss_release.py`'s documented
> degenerate identity). What those rows *do* newly exercise is the 8b **wrapper** —
> `catch/pretilt_hold` is raised and a 0 mm deferred reach is published — and that
> co-located path has **never run on hardware**. `TIER-D` below is the row that
> scores it, and it is why this section runs **before** you score anything else that
> throws.

**What actually changes at the machine.** The tier does not just relabel a goal — it
selects which of two shipped choreographies every toss goal runs:

| | Tier 8a (was default) | Tier 8b (is default) |
|---|---|---|
| POSITIONING | platform goes **level** to the nominated catch `(x, y)` | platform **pre-tilts IN PLACE** at the throw site `A` = its own LIVE commanded `(x, y)` (was the config site `(0, 0)` until 2026-07-29 — see § SECTION DISP) |
| throw | vertical | **tilt-aimed** at the displaced catch `B` |
| catch pre-tilt | the stock announcement pre-tilt runs | **suppressed** via `catch/pretilt_hold` |
| A→B platform reach | n/a | **deferred to `t_release`** — it happens in flight |
| CHECKING gates | tier, lead, flight band, event_vel, workspace | **plus** a known throw site, `\|B − A\| ≤` the closed-form reach bound, and a tilt clamp — all **before** the z-band check. (Read `\|B − A\| ≤ toss_max_displacement_mm` here until 2026-08-29; that cap and the lateral workspace box are both DELETED — see the § SECTION DISP banner) |

Direction of risk, stated honestly because it cuts both ways. *(HISTORICAL — the
cap is deleted; see the AMENDED banner below.)* The **gate** got
strictly tighter when 8b became the default: a goal accepted at `\|x\| ≤ 150 mm`
under 8a started refusing above the displacement cap. The **choreography** did not:
8b tilts the platform, suppresses the stock pre-tilt and puts a platform reach into
ball flight, none of which 8a does. Do not read "the gate is tighter" as "the motion
is smaller".

> **AMENDED 2026-07-29, again 2026-08-29 — read § SECTION DISP before running this
> section.** Premises moved twice. (1) The 70 mm cap became
> `jugglebot_operational.toss_max_displacement_mm` = 150 mm, and that key is now
> **DELETED** — the closed-form reach bound (256 mm at `T = 0.80 s`) is the only
> pre-throw `|B − A|` gate, so expect no cap refusal at any displacement this
> section drives. (2) The
> throw site `A` is the platform's **live commanded pose**, not the config `(0, 0)`,
> so **every displacement below is measured from wherever the platform is parked** —
> run each row from a `go_home`d platform or the numbers do not mean what they say.
> Both rows' PASS/ABORT text below has been re-pointed; the rows themselves still
> validate what they always did.

**Prerequisites — read the flash line separately, it is not the same step.**

1. **Build + relaunch (mandatory, and this is the step that fails silently):**
   ```bash
   cd ~/Desktop/Jugglebot/ros_ws \
     && colcon build --packages-select jugglebot_interfaces jugglebot \
     && source install/setup.bash
   ```
   then **relaunch**. The launch runs the *installed* copy of
   `jugglebot/hardware_config.py`; until this rebuild the robot executes **Tier 8a**
   while the repo, the tests, the logbook and this runbook all say 8b. That
   divergence is invisible from the Jetson except by reading the installed file,
   which is exactly what `TIER-PREREQ` does.
2. **No firmware flash for this change.** `grep -rn TOSS_TIER` over
   `ros_ws/src/jugglebot/Teensy_code_platform/`, `Teensy_code_canbridge/` and
   `CatchingCone_code/` (excluding the generated `hardware_config.h` itself) returns
   **0 hits** — no sketch reads the constant, even though three generated headers
   changed. Any flash this sitting needs comes from § Section FW, not from here.
3. **No config regeneration.** The six generated consumers are committed and were
   verified to be the deterministic output of the committed YAML.
4. **`jugglebot_interfaces` MUST still be rebuilt** — not for anything in *this*
   section, but because since 2026-07-29 `reload_coordinator_node` imports
   `TossContinuous` at module scope. Build the two-package command (matrix row B)
   in EVERY section of this runbook; a `jugglebot`-only build now raises
   `ImportError` before that node is constructed and takes `Reload`, `Toss` and
   `TossContinuous` down together. `ros2 action list | grep -c jugglebot/toss`
   returns 2 on a good install.

Standing session rules still apply — as amended: ~~power-cycle the can-bridge Teensy
immediately before the sitting~~ is **retired 2026-08-15** (the dispatch shift that
grew to `+57…78 ms` at ~94 min uptime was the RX-ring leak, fixed in FW 14 and
validated at 5.8 h and 15.2 h); still log `uptime_ms` with every timing number, and
check `gravity_correction_loaded` after the relaunch — `level` only if it reads
`false`.

### Recording for this section

Use `record:=true` **unchanged** (§ Recording) — `/catch/pretilt_hold` has been
in the record list since 2026-07-28 for `TIER-D` and is in the ONE list. Run the
trace recorder in its own terminal as always. `TIER-PREREQ` needs neither.

### ⚠️ THE ANALYSIS TRAP — do NOT use `--reject` on the refusal rows

`toss_trace_recorder.py check --reject` runs **RJ-1**, and RJ-1 hard-requires
**exactly one `REJECTED_NO_BALL`** line in the window (`check_rj1`:
`if len(nb) == 1: PASS else: FAIL`). `TIER-A` and `TIER-B` are refused for
*different* codes, so RJ-1 sees `len(nb) == 0` and prints

> `FAIL: 0 REJECTED_NO_BALL lines (expected exactly 1) … an earlier-code reject means
> that precondition wire is unhealthy; fix before the dry capture`

**on a completely correct capture** — and with an empty hint, because
`REJECTED_DISPLACEMENT` had no `REJECT_WIRE_MAP` entry until 2026-07-28. That is a
criterion firing on correct behaviour, the defect class the 2026-07-27 run close-out
existed to retire. RJ-1 is the *un-waived no-ball* check; it is not a general reject
decoder. **Score `TIER-A`/`TIER-B` from the node's own outcome line**, which is the
ground truth and needs no instrument:

```bash
grep -n "Toss REJECTED" ~/.ros/log/latest/reload_coordinator_node*.log | tail -5
```

`--reject` stays correct for its own row (§ Section HAND's un-waived no-ball
capture); it is only wrong here.

### CHECK TIER-PREREQ — the installed copy really is 8b (no robot, no bag, ~10 s)

Validates: that the build took. **Run this first; if it fails, score nothing below.**

```bash
grep -n "JB_OP_TOSS_TIER" \
  ~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/hardware_config.py
```

| quantity | PASS | ABORT (route back to this section) |
|---|---|---|
| the matched line | reads exactly `JB_OP_TOSS_TIER = '8b'` | reads `'8a'` — the `colcon build` did not take or the relaunch used a stale install. The robot is running Tier 8a while every artefact says 8b. Rebuild, relaunch, re-check. **Do not score any row below** |
| the grep | exactly **1** matching line | 0 lines — wrong install path or the package is not installed |

### CHECK TIER-A — a past-cap goal is refused, with no motion (**FLAG case**)

Validates: `catch-reach-degenerate-overshoot` Phase 4 / `single-ball-toss` Phase 4 —
that the 8b displacement cap is live and answers **before** the workspace box.
Refused in CHECKING, so **nothing actuates**. **Run it from a `go_home`d platform**
— the throw site is the live commanded pose, so `x = 200 mm` is a `200 mm`
displacement only from centre. That is `1.33×` the `150 mm` cap.

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 200.0, y: 0.0, z: 170.0}, throw_height_m: 0.8}" --feedback
```

| quantity | PASS | ABORT |
|---|---|---|
| node outcome line | `Toss REJECTED_DISPLACEMENT` | `REJECTED_WORKSPACE` — the build is stale (that is the **8a** answer to this goal), or the gate order moved. Re-run `TIER-PREREQ`. `REJECTED_POSE_UNKNOWN` — `trajectory/commanded_position` is not arriving; the throw site cannot be read and no displacement gate can run. Check trajectory_node is seeded + streaming, then re-run |
| goal terminal status | `ABORTED` | anything else |
| hand `pos_cmd` span over the whole goal, from the trace jsonl | **`0.0000 rev`** | any hand motion — a refused goal dispatched a stroke |
| commanded platform pose span (FK of `/leg_setpoint_echo`) over the goal | **`< 0.02°` and `< 0.2 mm`** | any platform motion — a refused goal reached POSITIONING |

### CHECK TIER-B — the workspace branch is still reachable (**FLAG case**)

Validates: the same two phases — that making the cap tighter did not make the
workspace gate **unreachable**. `x = 60 mm` is a *live* displacement: inside the
`150 mm` cap and inside the `256 mm` reach bound at `T = 0.8 s`, so the 8b gates
genuinely run **and pass** before `z` answers. `\|z − 170\| = 130 mm` against the
`±50 mm` band. Refused in CHECKING, so **nothing actuates**.

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 60.0, y: 0.0, z: 300.0}, throw_height_m: 0.8}" --feedback
```

| quantity | PASS | ABORT |
|---|---|---|
| node outcome line | `Toss REJECTED_WORKSPACE` | `REJECTED_DISPLACEMENT` — the cap has moved **below 60 mm** and is now swallowing every real displaced goal, leaving the workspace branch unreachable. Routes to `single-ball-toss` Phase 4. (Or the platform is not at centre — re-`go_home` and repeat) |
| hand `pos_cmd` span / platform pose span | **`0.0000 rev`** / **`< 0.02°`, `< 0.2 mm`** | any motion |

> Why the pair matters. `TIER-A` alone would pass even if the cap had collapsed to
> zero and started refusing *every* displaced goal; `TIER-B` is what detects that,
> because its 60 mm displacement must be **accepted** by the cap before `z` rejects.
> One row proves the gate fires, the other proves it does not fire too much.

### CHECK TIER-C — an in-band goal is NOT refused (**ACCEPT case, no motion**)

Validates: that the gates added by 8b do not refuse a goal 8a would have taken.
Send the ordinary centre goal **with the platform not yet armed into the streaming
hold** (i.e. before stage 4's arm), so it refuses on a *precondition* rather than on
an envelope gate. This is a **read of the reject code, not of the motion.**

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.8}" --feedback
```

| quantity | PASS | ABORT |
|---|---|---|
| node outcome line | any of `REJECTED_WRONG_MODE`, `REJECTED_NOT_LEVELLED`, `REJECTED_NO_BALL`, `REJECTED_HAND_*`, `REJECTED_MOCAP_STALE` — i.e. a **precondition** code. The envelope gates passed, which is the whole point | `REJECTED_TIER` — the config names a tier the FSM does not implement (re-run `TIER-PREREQ`). Or `REJECTED_DISPLACEMENT` — displacement is exactly `0 mm` here, so that code is impossible unless the cap or the throw site is corrupt. Both route to `single-ball-toss` Phase 4 |

> **This row is deliberately run un-armed.** Armed, the same goal is *accepted* and
> throws — see `TIER-D`. Do not run it armed and expect a static row.

### CHECK TIER-D — the co-located 8b throw (**THIS ONE ACTUATES — first ever on hardware**)

Validates: `catch-reach-degenerate-overshoot` Phase 4 — the 8b wrapper at
`\|B − A\| = 0`. **Every one of the 11 validated Tier-8b throws on 2026-07-27 was
displaced; this path has no hardware coverage at all.** It is also the shape every
row *above* this section sends, so run it **before** re-scoring any of them.

Normal toss preconditions: ball seated and confirmed by eye, area clear, armed into
the streaming hold, bag + trace recorder running. One attempt is enough to score the
wrapper; take 3 if you want a rate.

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| quantity | PASS | ABORT |
|---|---|---|
| `catch/pretilt_hold` transitions across the goal | exactly **one** `True` then exactly **one** `False`, the `False` **at or before** the terminal | still `True` after the terminal — the gate is stranded and the **next** catch runs with its pre-tilt suppressed. Stop the sitting; routes to `single-ball-toss` Phase 4 |
| first `catch/dynamic_target` timestamp, relative to `t_release` | **`+0.000 s` to `+0.100 s`**, never negative (measured band 2026-07-27: `+0.013…+0.050 s` on 11/11) | **any value < 0** — the deferred reach fired *before* release. On a displaced throw that un-tilts the platform mid-launch and destroys the aim; here it is the same defect with the aim error masked by `\|B − A\| = 0` |
| deferred reach travel, `catch/dynamic_target` first→last | **`≤ 2.0 mm`** (nominal 0 mm at `B == A`) | `> 2.0 mm` — a zero-displacement goal is commanding real platform travel in flight |
| commanded pose span (FK of `/leg_setpoint_echo`) over `[release − 0.10 s, release]` — `levelling_tilt_bag_check.py --t0 <release−0.10> --t1 <release> --plateau-min-s 0.05 --plateau-tol 1.0`, read `span_deg` and the `commanded position span (x,y,z) mm` line | **`< 0.02°` and `< 0.2 mm`** (measured `0.0000° / 0.0000 mm` on 11/11 on 2026-07-27) | any commanded motion through release |
| hand peak `pos_meas` | `< 10.5 rev` at 0.78 m | `≥ 10.5 rev` — see the end-stop amendment in § THE RUN SHEET's EXECUTED box. **No tosses above 0.78 m in THIS section.** Going above 0.78 m has exactly one route and it is not here: § CHECK HAND-7's ladder, on a board FW-1 reads as current (`v3`), in its own capture |
| caught | by eye; not a gate on one attempt | — |

Score the pre-release **half** window only: a reach that starts *at* release
necessarily puts motion into the second half of a symmetric `release ± 0.10 s`
window, so the symmetric form is unsatisfiable by construction (recorded as an
instrument defect on 2026-07-28).

### CHECK TIER-E — the displaced throw still works (**ACTUATES; regression watch**)

Validates: `single-ball-toss` Phase 4 — a straight repeat of the already-successful
2026-07-27 T4 geometry, now on the default build rather than a hand-flipped one.
Aim into `−x` at `T ≥ 0.80 s`. **Run it from a `go_home`d platform** — the throw
site is the live commanded pose. (The `−x` choice came from the 2026-07-25 asymmetry
map, which is **stale vintage** — it was measured on the pre-2026-07-26 moving-rim
machine. Keep `−x` here only so this row stays a like-for-like repeat of the
2026-07-27 T4 geometry; the live directional evidence is § SECTION DISP's re-run map.)

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: -70.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| quantity | PASS | ABORT |
|---|---|---|
| goal accepted | not refused — `70 mm` is far inside the `150 mm` cap | `REJECTED_DISPLACEMENT` at 70 mm — the cap collapsed, or the platform was not at centre when the goal was sent. Routes to `single-ball-toss` Phase E |
| first `catch/dynamic_target` vs `t_release` | **`+0.000 s` to `+0.100 s`**, never negative | any value `< 0` |
| deferred reach travel | **`50–75 mm`** (measured `51.5–72.2 mm` on 2026-07-27) | `< 50 mm` — the reach is not spanning A→B |
| commanded pose span over `[release − 0.10 s, release]` | **`< 0.02°` and `< 0.2 mm`** | any commanded motion through release |
| caught, over 3 attempts | **≥ 2/3** | `0/3` — routes to `single-ball-toss` Phase 4's asymmetry work, **not** to this section |

### Not in this section

- **Whether the 8b default is the right choice.** It is an operator decision taken on
  the 2026-07-27 T4 evidence and recorded in
  `logbook/2026-07-28-toss-tier-8b-default.md`. These rows check that it *deployed*
  and *behaves*, not that it is wise.
- **Raising the cap, deriving the throw site from the current pose, and the
  reach-envelope work.** That was decision (d)'s displaced-throw programme and it
  **landed 2026-07-29** as `single-ball-toss` Phase E — see § SECTION DISP, which is
  where its rows live. This section keeps its own rows because they validate the
  *tier default deployment*, which is a separate question from the displaced-throw
  capability.
- **The T0–T4 capability ladder.** `tests/hardware/session_phase8_toss_hardware.md`,
  which runs **after** this file — and note its new § TIER banner: `T0–T3` were
  written for Tier 8a and `T3` in particular does not mean what it says on an 8b
  build.
- **`REJECT_WIRE_MAP` hygiene.** `REJECTED_DISPLACEMENT` and `REJECTED_TILT_CLAMP`
  were added on 2026-07-28 and the `REJECTED_TIER` hint corrected; `REJECTED_POSITION`
  landed 2026-08-29 (with the subcode ladder — `BUSY` / `NO_RESPONSE` /
  `WIRE_DISARMED` / `WORKSPACE` — since that code always names one). That backlog
  is owned by `logbook/2026-07-25-toss-rejected-not-levelled.md` § Follow-ups.
- **Reject codes now carry their numbers (2026-08-29).** Every *limit-bearing*
  refusal appends a parenthetical naming the requested value, the limit it broke
  and the knob that moves it — `REJECTED_WORKSPACE(B.z 225.0 mm is 55.0 mm >
  band 50.0 mm [TOSS_Z_BAND_MM] …)`. (This example read `|B.y| = 178.0 mm >
  160.0 mm [toss_workspace_xy_mm]` until 2026-08-29, when the lateral box was
  deleted and the z band became the only `WORKSPACE` clause.) **Score every row
  in this file on the CODE**, i.e.
  on the text before the first `(`; the parenthetical is diagnosis, never
  identity. Structural verdicts with no numeric knob (`NOT_LEVELLED`,
  `WRONG_MODE`, `NO_BALL`, `HAND_*`, `MOCAP_STALE`, `TIER`, `POSE_UNKNOWN`, the
  `BB_*` family) stay bare and their rows are byte-unchanged.

---

## SECTION POSS — the possession verdict (contract C-POSSESS-1)

> **⚠ AMENDED 2026-08-10 — READ BEFORE SCORING ANY ROW BELOW.** The **hand ball
> sensor is now the PRIMARY possession source** and the tracker is demoted to the
> arrival corroborator (`logbook/2026-08-10-sensor-truth-possession.md`,
> `plans/active/catch-robustness.md` Phase 1, contract §§ 2.1 / 3.2 / 3.3). Three
> consequences for this section, all of them scoring-critical:
> 1. **`POSS-1.3` and the reload half of the top-level `POSS-1` row are
>    INVERTED.** A reload reading `CAUGHT` was the ABORT and is now the expected
>    PASS. Both rows are rewritten in place; do not score them from memory.
> 2. **Two rows are new**: `POSS-1.7` (the reload's `CAUGHT` terminal executes for
>    the first time — REPORT, ungated on purpose) and `POSS-1.8` (the `UNKNOWN`
>    paths, which are test-only in this build).
> 3. **The ball-evidence gate is ON** (`toss_require_ball_evidence: true`), so a
>    toss into a valid-empty cup is `REJECTED_NO_BALL` and one whose sensor cannot
>    answer is `REJECTED_BALL_UNKNOWN`. Both are the gate working.
>
> Everything the section says about the TRACKER's numbers (the 70 mm bound, the
> deleted z bound, the split-track signature) is unchanged and still scored — the
> tracker just no longer decides.

**What landed, and why it needs a bench row at all.** The coordinator's CAUGHT
gate ANDed two spatial bounds, and the vertical half — `|z − catch_z| ≤ 150 mm` —
could not be satisfied by a real catch. The tracker declares CAUGHT *because the
mocap marker vanished*, so the position it publishes at that instant is a
dead-reckoned free-fall extrapolation from the last sighting, frozen thereafter.
Measured across the 2026-07-27 sitting's **17 self-tosses, every one a catch the
operator watched land**: `xy` error **0.30 – 3.88 mm** against `z` error
**305 – 1007 mm**. All 17 failed. `success` was False by construction on every
ball op the machine has ever run, which is why standing rule 3 used to say the
tracker "still reports MISSED on real catches".

The z bound is deleted (REPORT-only, forever) and the horizontal bound is now the
catching structure's own entry aperture, `GEOM_ARM_RADIUS_MM = 70 mm`, instead of
`200 mm`. Full reasoning: `ros_ws/docs/ball_possession_contract.md`.

**This row is a REPORTING check.** Nothing here changes what the robot is
commanded to do at the moment of a catch. It does change *when a caught toss
terminates* — see § The one behavioural change below, which is the part to watch.

### Deployment

`colcon build --packages-select jugglebot_interfaces jugglebot` **+ relaunch**.
No config regeneration, **no firmware flash**. Nothing in *this* section changes an
interface, but the two-package build is MANDATORY in every section since
2026-07-29: `reload_coordinator_node` imports `TossContinuous` at module scope, so
a `jugglebot`-only build raises `ImportError` before that node is constructed and
takes `Reload`, `Toss` and `TossContinuous` down together (matrix row B). The
launch runs the *installed* copy, so without the relaunch the machine reproduces
the old always-MISSED behaviour exactly and this row scores a false ABORT.

Confirm the installed copy carries it, before the sitting:

```bash
INST=ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot

# 1. The NEW module must exist in the install tree. It is a new file, so a
#    partially-cached build is the one way this lands half-applied — and the node
#    would then fail to import, taking the whole launch down rather than
#    misbehaving quietly.
ls -l $INST/ball_possession.py

# 2. The bound must be the geometry-sourced one, not the old 200.0 literal.
grep -n "GEOM_ARM_RADIUS_MM" $INST/reload_coordinator_node.py
# expect one hit, on the _CAUGHT_MAX_XY_ERROR_MM assignment.
# no hit  => the install is stale; rebuild before capturing anything.

# 3. Belt and braces — the deleted bound must be gone.
grep -c "_CAUGHT_MAX_Z_ERROR_MM" $INST/reload_coordinator_node.py   # expect 0

# 4. ADDED 2026-08-10. The SENSOR half — the part that inverts POSS-1.3 and turns
#    the ball-evidence gate on. Without it the section silently re-measures the
#    tracker-only era and you score the inverted rows against the old plant.
grep -c "HandBallSensorSource" $INST/ball_possession.py             # expect >= 1
grep -c "REJECTED_BALL_UNKNOWN\|BALL_UNKNOWN" $INST/toss_sequencer.py  # expect >= 1
python3 -c "import sys; sys.path.insert(0, '$INST/..'); import jugglebot.hardware_config as h; print(h.JB_OP_TOSS_REQUIRE_BALL_EVIDENCE, h.JB_BD_ARRIVAL_LEAD_S, h.JB_BD_ARRIVAL_WINDOW_S, h.JB_BD_RETENTION_WINDOW_S)"
# expect: True 0.2 1.5 1.5   — anything else and the windows you score are not the
# windows the robot ran.
```

### CHECK POSS-1 — the gate's verdicts against the operator's eyes

Runs off the CAP-WORK capture; no extra actuation. Score it *after* the sitting,
from the log lines and the bag.

```bash
# 1. The per-attempt verdict lines the node emits (one INFO per ball per verdict).
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep -c "possession CONFIRMED" "$LOG"
grep -c "possession REFUSED"   "$LOG"
grep -c "SENSOR_BLIND"         "$LOG"     # ADDED 2026-08-10 — expect 0 (POSS-1.8)
grep    "Ball .*: possession"  "$LOG"     # read them; each names its numbers
# The REFUSED lines now have TWO authors and say which: "the cup sensor did not
# observe the ball arrive" (the sensor vetoed — since 2026-08-10 the commonest
# refusal on a self-toss the tracker still likes) vs "arrival N mm > 70 mm from
# the catch point" (the tracker refused, sensor blind or absent). Read the words,
# not just the count — they route to different subsystems.

# 2. The authoritative per-attempt outcomes.
grep -c "Toss CAUGHT"   "$LOG"
grep -c "Toss MISSED"   "$LOG"
grep -c "Reload CAUGHT" "$LOG"            # ADDED 2026-08-10 — no longer always 0
grep -c "Reload MISSED" "$LOG"
grep -c "REJECTED_BALL_UNKNOWN\|REJECTED_NO_BALL" "$LOG"   # the live gate refusing

# 3. The offline verdict on the same bag (venv — standing rule 5).
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python tools/probes/possession_verdict_bag_check.py \
       --bag ~/Desktop/rosbags/<CAP-WORK stamp>
# ADDED 2026-08-10 — the SENSOR half, which is what decides now. Sibling probe:
python tools/probes/hand_sensor_verdict_replay.py \
       --bag <CAP-WORK stamp> --merge-tracker refuse --json
```

`possession_verdict_bag_check.py` prints one row per destination-tagged track with
its arrival error, its REPORT-only plane drop, and the TRACKER's verdict — which
since 2026-08-10 is the *corroborator*, not the decision, so a disagreement with
the live log is a finding only where the sensor was UNKNOWN. `hand_sensor_verdict_replay.py`
replays the deciding half; note its `BOUNCE` label is a live `CAUGHT` (it scores
with both windows closed, the coordinator answers on the first confirmed tick).

| # | quantity | PASS | ABORT |
|---|---|---|---|
| POSS-1.1 | self-toss `Toss CAUGHT` count vs your by-eye catch count | **equal** | gate count `< by-eye − 1` — a real catch scored MISSED. Route to this section, and record the ball's arrival error from the probe |
| POSS-1.2 | self-toss `CAUGHT` on a ball that **never arrived at the cup** (a wide miss — the probe shows it as a large arrival error) | **zero** | any. This is the false-positive direction and it outranks POSS-1.1 — stop and record the estimate |
| POSS-1.2b | self-toss `CAUGHT` on a ball that **arrived and then left** (entered the cup, then bounced/rolled out) | **REPORT — do not abort.** Record the ball id, the arrival error, and what you saw | *(no ABORT)* — this is specified behaviour, not a fault: the tracker source cannot observe RETENTION and the contract forbids it from claiming otherwise (C-POSSESS-1 § 2, § 7). The count you write here **sizes the ball-in-cup sensor work**, which is what closes it |
| POSS-1.3 | reload `Reload CAUGHT` count **⚠ INVERTED 2026-08-10 — read this row, do not score it from memory** | **== your by-eye reload catch count.** Until 2026-08-10 the pass was **0** with one `possession REFUSED` per attempt, because every destination-tagged reload track is a split track 204.9–752.9 mm out and the TRACKER refused catches the operator watched land. The hand ball sensor is primary now: it reads the cup, so the split track no longer suppresses the verdict | a reload catch you watched land still reading `MISSED` ⇒ either the sensor half is not deployed (re-run the deployment greps — `HandBallSensorSource` must be in the installed `ball_possession.py`) or the arrival edge fell outside the window: record `catch_dt` off the outcome line and route to `catch-robustness` Phase 1. A `CAUGHT` on a reload that visibly did **not** land in the cup is the false-positive direction and outranks it |
| POSS-1.4 | log discipline | exactly **one** `Ball N: possession …` line per (ball, verdict); all at INFO | duplicates, or any at WARN/ERROR |
| POSS-1.5 | probe vs live agreement | the probe's `CAUGHT` count **==** the log's `possession CONFIRMED` count | any difference ⇒ the installed copy is stale (re-run the deployment grep above) |
| POSS-1.6 | **reload arrival errors, watched not gated** — the probe's `arrival_mm` column for reload attempts | today: **204.9 – 752.9 mm** ("all refused" **meant the TRACKER refused**, and since 2026-08-10 that no longer suppresses the verdict — the same numbers now print as REPORT-only cross-check on a line that can still read `CAUGHT`). Just record the range | *(no ABORT)* — but if any reload arrival error lands in the **30 – 100 mm** band, **stop and read this**: that is the signature of the tracker mis-association *healing*, and the 70 mm bound is **knowingly under-sized** for a healthy reload path. The reload era's real-marker tracks measure **34.4 / 34.9 / 37.6 / 68.4 mm**, so a genuine reload catch sits **1.6 mm inside** the bound — a 1.02x margin, plus up to 80 mm of catch-reach displacement the reference point does not follow. Route to `ros_ws/docs/ball_possession_contract.md` § 4; the bound must be re-derived by the tracker phase, **not** nudged at the bench |
| POSS-1.7 | **NEW AND UNGATED, 2026-08-10 — the reload's `CAUGHT` terminal executes for the first time in the machine's history.** A successful reload runs `ACTION_RECENTER`: lower the catch latch + `go_home`, deliberately **no** hand retract (the hand is holding the ball). Every reload ever run terminated `SAFE_ABORT` instead, because the tracker refused every reload catch by construction — so `POSS-1.3` flipping is what makes this path live | **REPORT — no PASS/ABORT is set here on purpose.** Record three things: (a) does the `go_home` after a caught reload behave like LVL-2 — same profiled move, no step rejection; (b) does the BALL stay in the cup through it (mocap ball marker within `GEOM_HAND_RADIUS_MM` = **35 mm** of the cup axis) — this is POSS-2.4's question on the reload path; (c) is the hand left inside the **±0.5 rev** park band the next goal's `hand_parked` precondition needs | a `MAX_DEVIATION` or guard E-STOP during that `go_home` is a **hard stop for the section**. Nothing else here aborts: the row deliberately sets no threshold, because the path has no measured baseline and gating it is an operator decision (`plans/active/catch-robustness.md` Phase 1 open items). If you want zero new risk on the first run, do the reload rungs with `go_home` issued manually and score (b) on the held pose first |
| POSS-1.8 | **the blind-sensor paths, which are TEST-ONLY in this build** (203,922 real samples across three bags were 100 % `ball_held_valid`). Two operator-visible signatures, and they are NOT the same line: (a) `REJECTED_BALL_UNKNOWN` on a *toss goal* — the live `evidence()` read at CHECKING could not answer; (b) `SENSOR_BLIND` inside the `[reason]` bracket of a possession line — the verdict silently fell back to the tracker (a bare `possession UNKNOWN` line is effectively unreachable through today's caller: it only runs on a tracker CAUGHT, and the tracker always has an estimate to fall back to) | **zero of each** on a healthy sensor. To exercise it deliberately, **kill the SDO poller, not the link**: `hand_fresh` gates *before* `ball_seated`, so dropping the whole bridge gives `REJECTED_HAND_STALE`, not `REJECTED_BALL_UNKNOWN` | *(no ABORT on the deliberate test)* — but either signature during normal running is a **finding**: record the surrounding `ball_held_valid` stream and route to `plans/archived/hand-ball-sensor.md`. A goal refused `REJECTED_BALL_UNKNOWN` is the gate working (fail-closed by design, deliberately NOT BallButler's fail-open boot default); `toss_require_ball_evidence: false` is the documented total bypass if you need to finish a sitting |

Record the raw counts either way. This is the row that retires "judge by eye"
across the whole file, and it cannot be retired on one sitting.

### The one behavioural change, and what to watch

Both FSMs already branch on the verdict (`toss_sequencer._step_in_flight`,
`reload_sequencer._step_in_flight`): a confirmed catch finishes the goal
immediately, an unconfirmed one runs to the settle deadline and
finishes with `SAFE_ABORT`. **That branch has been dead on hardware for the
machine's whole life.** From this build it is live on every successful self-toss:

| | before | after |
|---|---|---|
| terminal instant | landing + **0.70 s** (`catch_confirm_window_s`; **0.80 s** from 2026-08-21, when it became derived from `ball_possession.ARRIVAL_BAND_MAX_S`, and **0.56 s** since 2026-08-24, when that band was re-measured on the post-FW-14 plant — the row below is the 2026-07-28 measurement and is left at the value it was taken against) | the CAUGHT tick — measured landing + **0.202 – 0.442 s**, median **0.209 s**, over the 17 self-tosses |
| terminal action | `SAFE_ABORT` — **retract** the hand, lower latch, `go_home` | `STAY` — lower latch, **no retract, NO `go_home`** (2026-07-29; it was `RECENTER` = lower latch + `go_home` between the C-POSSESS-1 build and this one) |

> **AMENDED 2026-07-29 — the CAUGHT terminal no longer calls `go_home`.**
> `jugglebot_operational.toss_stay_at_pose_on_caught` ships `true`, so a caught
> toss holds its catch pose and the next toss throws from there (§ SECTION DISP).
> That **retires the risk POSS-2.3 and POSS-2.4 were written to score** — the
> earlier-`go_home`-with-a-ball-in-the-cup path does not execute on a caught toss
> at all now — and replaces it with a different one, scored as **DISP-5**: the
> platform holds the catch's RECEIVE TILT indefinitely instead of returning to
> level. Run POSS-2.3/2.4 only if you set the key `false`; both rows are marked
> accordingly below.

Checked offline against the 2026-07-27 capture before this shipped: the catch
stroke has already finished by the CAUGHT tick — hand `pos_meas` sits within
**±0.045 rev** of the retract target on all 17 (range `−0.045 … +0.026`), where
0.3 s earlier 7 of the 17 were still descending through **0.30 – 3.10 rev** — so
the earlier terminal cannot interrupt a moving hand, and skipping the retract
leaves the hand inside the `±0.5 rev` bottom park band the *next* goal's
`hand_parked` precondition needs (worst excursion over the following 3 s
**0.069 rev**, a 7.2x margin).

| # | quantity | PASS | ABORT |
|---|---|---|---|
| POSS-2.1 | a Toss immediately following a **caught** Toss | accepted — no `REJECTED_HAND_NOT_PARKED` | any such rejection ⇒ the CAUGHT terminal is leaving the hand off the park band. Recover with a Reload (which primes and retracts) and record the `pos_meas`. **Note under STAY the second goal is also a live § SECTION DISP chaining test** — its throw site is wherever the first caught |
| POSS-2.2 | hand `pos_meas` at the caught toss's terminal | within **±0.5 rev** of `JB_OP_HAND_RETRACT_REV`; expect **\|pos\| ≤ 0.10 rev** (offline worst case 0.069) | `> 0.5 rev` |
| | *(provenance, if you need to re-derive the 0.069)* | The offline hand-state-after-CAUGHT analysis was a **one-off `/tmp` probe and is not committed**, so this number cannot be re-run from `tools/probes/`. To re-derive: read `/hand_telemetry` `pos_meas` from `~/Desktop/rosbags/2026-07-27_15-39-38` over the 3 s following each self-toss track's first `CAUGHT` sample (track ids ≥ 80; `possession_verdict_bag_check.py` prints the ids). If you find yourself needing it more than once, promote it — CLAUDE.md's probe rule puts reusable harnesses in `tools/probes/` committed | *(not a check)* |
| POSS-2.3 | **ONLY IF `toss_stay_at_pose_on_caught` is `false`.** The `go_home` after a caught toss | behaves as LVL-2 (same profiled move, ball seated) | any step rejection, `MAX_DEVIATION`, or guard latch. At the shipped `true` there is no `go_home` to score — **N/A, and seeing one is itself the finding**: it means the installed copy is stale |
| POSS-2.4 | **ONLY IF `toss_stay_at_pose_on_caught` is `false`.** Does the BALL survive the earlier `go_home`? The ball marker (mocap) at `go_home` completion, relative to the platform's cup axis | still within **35 mm** (`GEOM_HAND_RADIUS_MM`) of the cup axis, on **every** caught toss | the ball leaves the cup during `go_home`. **This is the row that retires an assertion, so run it deliberately.** § 5 of the contract argues this move is "same class as today" from hand `pos_meas` — which says where the *hand* is, not whether the *ball* has come to rest. A caught toss now starts `go_home` **0.26 – 0.50 s earlier**, essentially at the instant the catch stroke arrests the ball, where before it had the full 0.70 s settle window **plus** a retract. This path has never executed on hardware. If you want zero new risk on the first run, ask for the decoupling in the contract's § 5 residual 2 (resolve the verdict early, hold `go_home` to the old deadline) — it lands the reporting fix with no timing change |

`catch/prime_hold` is also released **0.26 – 0.50 s** earlier, re-opening
`catch_coordinator`'s auto-prime with a ball in the cup. **This is not new**:
today's `SAFE_ABORT` path releases the same hold at the settle deadline with the
ball equally seated, after retracting the hand to the same ~0.0 rev the catch
trajectory already reached. Same state, up to half a second earlier. Watch for an
unexpected hand ascent after a caught toss and record it if you see one.

### Not in this section

- **Fixing the tracker.** The split-track corruption is an open investigation. Its
  measured signature is in `ros_ws/docs/ball_possession_contract.md` § 4 and in
  `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`. POSS-1.3 passing at
  **0 CAUGHT reloads** is this contract working, not the corruption persisting
  unnoticed.
- **The ball-in-cup hand sensor.** Installed 2026-07-28; no code exists for it.
  When its plumbing lands it becomes the PRIMARY possession source and is the only
  thing that can answer *retention* — did the ball stay in the cup. Until then the
  tracker source reports retention `UNKNOWN` and every log line says so.
- **Turning on `toss_require_ball_evidence`.** Still `false`, deliberately and
  unchanged. It is a precondition that can refuse a goal, and it belongs to
  whoever validates the sensor (sitting logbook, decision row (e)).

  > **⚠ BOTH SUPERSEDED 2026-08-10** (`catch-robustness` Phase 1,
  > `logbook/2026-08-10-sensor-truth-possession.md`). The sensor's plumbing landed
  > and it IS the primary source; the gate defaults `true`, so a valid-empty cup is
  > `REJECTED_NO_BALL` and a blind sensor is `REJECTED_BALL_UNKNOWN`. These two
  > bullets are kept because they are what the rows below were scored against —
  > a POSS row read against them is still being read correctly.

---

---

## SECTION DISP — displaced throws to ±150 mm (`single-ball-toss` Phase E)

> **Appended 2026-07-29.** This is operator decision **(d)** of 2026-07-28 —
> *"throw across large translations, at least ±150 mm at z = 170, from oblique
> platform positions, with the platform STAYING at its catch pose so sessions chain
> A → B → C"* — implemented as one phase because its four parts interlock. Validates
> `plans/active/single-ball-toss.md` **Phase E**. Run it **after** § SECTION TIER
> (which proves the 8b wrapper deployed) and after § SECTION POSS (whose verdicts
> are the instrument every rung below is scored with).
>
> It **changes premises in sections above**; those have been re-pointed in place and
> each carries an AMENDED note. If you are reading a printed copy, the three that
> moved are § SECTION TIER's cap and throw-site rows, § SECTION POSS's `POSS-2.3` /
> `POSS-2.4` (now conditional), and the Reload precondition list.
>
> **⚠ AMENDED AGAIN 2026-08-29 — BOTH POLICY KNOBS ARE DELETED.** This whole
> section was written around a `150 mm` cap and a `±150/160 mm` planning box.
> Neither exists: `toss_max_displacement_mm` and `toss_workspace_xy_mm` were
> removed by owner decision. What that means for running the ladder:
>
> * The **rungs below are unchanged and still the right ladder** — 70 → 100 →
>   150 mm is now a DIFFICULTY progression the operator holds to, not one the
>   machine enforces. Nothing will stop you skipping to 150; do not.
> * The surviving pre-throw `|B − A|` gate is the **closed-form reach bound**
>   (256 mm at `T = 0.80 s`, 108 at `0.60`, 83 at `0.55`), which follows the live
>   `trajectory/set_limits` limits. At the ladder's `T = 0.78-0.80 s` it does not
>   bind below ~250 mm, so **expect NO pre-throw refusal at any rung here**.
> * A far-LATERAL goal is no longer `REJECTED_WORKSPACE`. On the **shipped
>   Tier 8b** it is refused at CHECKING as
>   `REJECTED_POSITION(<planner code>: the nominated catch pose … is not
>   reachable …)` — the build-time reach-feasibility gate (2026-08-29 audit fix:
>   8b pre-positions at A and defers the A→B reach, so `go_to_pose` never judges
>   B). On Tier **8a** it reaches POSITIONING and is refused
>   `REJECTED_POSITION(UNREACHABLE)` by `go_to_pose` itself. Same base outcome,
>   different phase and subcode — read the subcode to know which answered.
> * `REJECTED_CHAIN_UNREACHABLE` is **gone**. The FORMER KNOWN LIMITATION box
>   above DISP-6 is now purely historical — a cap-edge chain is admitted, and no
>   refusal of that class can appear at all.

### What changed, in one table

| | before (the 2026-07-27 build) | now |
|---|---|---|
| throw site `A` | config `jugglebot_operational.toss_throw_site_mm` = `(0, 0)` | the platform's **LIVE commanded pose**, read from the new `trajectory/commanded_position` topic. The config key is **retired** |
| displacement cap | `70 mm`, hard-coded | `jugglebot_operational.toss_max_displacement_mm` = **`150 mm`**, operator-adjustable — ⚠ **DELETED 2026-08-29**; the closed-form reach bound is now the only pre-throw `\|B − A\|` gate (256 mm at `T = 0.80 s`, 108 at `0.60`, 83 at `0.55`) and it follows the live `set_limits` |
| catch reach envelope | `80 mm` around the pose held at arming — which also capped **requested** reach | `80 mm` around the **declared catch point B** (contract **C-REACH-1**, `ros_ws/docs/catch_reach_envelope.md`). Drift is bounded exactly as tightly; requested reach is not |
| CAUGHT terminal | `RECENTER` — lower latch + `go_home` | **`STAY`** — lower latch, release the holds, **no `go_home`**. `jugglebot_operational.toss_stay_at_pose_on_caught` = `true` |
| MISSED / ABORT terminals | `SAFE_ABORT` (retract, latch down, `go_home`) | **unchanged** |
| Reload preconditions | mode, BB, mocap, streaming | **plus `REJECTED_NOT_CENTERED`** — see DISP-7 |

**The failure this closes, and why it had to close before the cap could move.**
On 2026-07-27, bag `2026-07-27_16-07-30`, four Tier-8b goals at `(100,100)`,
`(90,90)` and `(80,80)×2` — nominal displacements `141 / 127 / 113 / 113 mm`, run
under an uncommitted 250 mm cap override — were **all four refused**, `catch target
146/131/117/117 mm from the armed hold pose exceeds the 80 mm reach envelope`. 4/4.
The refusal itself is fine; **when** it arrived is not. The deferred A→B reach is
published at `t_release`, so every one of those rejections landed with the ball
already airborne and the platform holding at `A`. Nothing recovers from that.
Raising the cap without moving the envelope centre would have re-run that failure at
150 mm.

### Deployment for this section — read all three lines

1. **`colcon build --packages-select jugglebot_interfaces jugglebot` + relaunch.**
   Mandatory. The launch runs the *installed* copy; until the rebuild the robot
   runs the **70 mm cap, the config throw site and the `go_home` terminal** while
   this file says otherwise.
2. **The `jugglebot_interfaces` half is mandatory too, though not for this
   section's own sake.** Both topics this section adds
   (`trajectory/commanded_position`, `catch/reach_center`) are
   `geometry_msgs/Point` — deliberately, so *this* change cannot create a split
   interface build. But since 2026-07-29 `reload_coordinator_node` imports
   `TossContinuous` at module scope, so a `jugglebot`-only build raises
   `ImportError` before that node is constructed and takes `Reload`, `Toss` and
   `TossContinuous` down together (matrix row B). The DISP-0 greps below read
   `install/jugglebot/...` only and will PASS on that stale build —
   `ros2 action list | grep -c jugglebot/toss` (expect **2**) is what detects it.
3. **No firmware flash and no config regeneration** *for this section*. (§ Section
   FW's Platform Teensy flash and § CHECK HAND-7's are separate and still owed.)

### Pre-flight DISP-0 — the installed copy really is Phase E (no robot, no bag, ~20 s)

```bash
INST=~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -c "JB_OP_TOSS_MAX_DISPLACEMENT_MM" $INST/hardware_config.py          # expect 0
grep -c "JB_OP_TOSS_WORKSPACE_XY_MM" $INST/hardware_config.py             # expect 0
grep -n "JB_OP_TOSS_STAY_AT_POSE_ON_CAUGHT" $INST/hardware_config.py
grep -c "JB_OP_TOSS_THROW_SITE_MM" $INST/hardware_config.py            # expect 0
grep -c "ACTION_STAY" $INST/toss_sequencer.py                          # expect >= 2
grep -c "catch/reach_center" $INST/trajectory_node.py                  # expect >= 1
grep -c "trajectory/commanded_position" $INST/reload_coordinator_node.py  # expect >= 1
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-0.1 | the retired cap + box, and the STAY flag | both `grep -c` are **`0`**, and `JB_OP_TOSS_STAY_AT_POSE_ON_CAUGHT = True` | either count `1` ⇒ the install PREDATES the 2026-08-29 deletion; **stop**, rebuild, relaunch. (This row read the opposite until then — it required `JB_OP_TOSS_MAX_DISPLACEMENT_MM = 150.0` to be present) |
| DISP-0.2 | the retired key | `0` | `1` ⇒ the install predates Phase E; **stop**, rebuild, relaunch |
| DISP-0.3 | the three greps | all non-zero | any zero ⇒ stale install |

Then, with the graph up and **before** anything actuates:

```bash
ros2 topic echo /trajectory/commanded_position --once
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-0.4 | one message arrives within ~1 s, `x/y/z` matching where the platform is holding (`z ≈ 170` at ACTIVE) | yes | **silence** ⇒ trajectory_node is not seeded/streaming. Every 8b goal will read `REJECTED_POSE_UNKNOWN` until it is. That refusal is correct and is the fail-closed design; fix the graph, do not work around it |

### THE LADDER — climb it in order, and do not skip a rung

Every rung is a `Reload` (to load a ball) followed by the `Toss` shown. Score each
with the § SECTION POSS instruments — the live `Toss …` outcome line, your own eyes,
and afterwards:

> **`go_home` BEFORE every `Reload` from DISP-3 onward.** Under `STAY` a caught
> toss leaves the platform parked at its catch pose, and the Reload precondition
> tolerance is **`66.5 mm`** — so after any caught displaced toss at `≥ 70 mm` the
> next rung's `Reload` will correctly refuse `REJECTED_NOT_CENTERED` until you
> re-centre. **That refusal is the gate working, not a fault.** It is deliberately
> tighter than the `80 mm` reach envelope: the reload's own pre-tilt shifts the
> commanded centroid a saturated `13.47 mm` off the catch point
> (`64.78·sin(12°)`, and `compute_catch_orientation` clamps at `12°` for every
> real BB arrival), so a tolerance of `80` would have ADMITTED parks in
> `(66.5, 80] mm` and then rejected the pre-tilt `WORKSPACE` **after BB had been
> asked to throw** — the DISP-7.1 E-STOP condition, reached through a gate that
> said yes.
>
> **Do NOT send a `Toss` while a move is still executing.** The throw site `A` is
> SAMPLED once at goal accept from a 5 Hz topic with a 1.0 s staleness window, and
> nothing refuses a goal issued mid-traverse. A goal sent while the platform is
> still driving nominates a pose it has **already passed**, and POSITIONING then
> supersedes the in-flight move with a profiled reversal *back* to it — an
> unrequested backtrack of up to ~200 mm at the typical 0.2 s sample age. It looks
> correct in the log (the aim stays self-consistent), so the only defence is the
> discipline: wait for the move to finish, confirm with
> `ros2 topic echo /trajectory/commanded_position --once`, **then** send the Toss.

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate && cd ~/Desktop/Jugglebot
python tools/probes/possession_verdict_bag_check.py --bag ~/Desktop/rosbags/<BAG>
python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/<BAG> --csv
```

`ball_arrival_offset.py` is the rung's real measurand: it says **where the ball
actually arrived** relative to the announced landing, which is the number that
decides whether a displaced throw is aimed or merely accepted. Pass `--ref X Y Z`
with the rung's nominated `B` when scoring `possession_verdict_bag_check.py` on a
displaced rung — the default reference is the ACTIVE catch point and would score
every displaced catch as a wide miss.

**Standing per-rung ABORT, applies to all of them.** Any `WORKSPACE` line naming the
reach envelope in the `trajectory_node` log during a flight ⇒ **stop the ladder**.
That is the 2026-07-27 mid-flight failure recurring, and it means the
`catch/reach_center` declaration is not reaching trajectory_node. Check for
`toss declared catch reach centre (…)` in the coordinator log and
`reach envelope 80 mm about (…) [declared catch/reach_center]` in the
trajectory_node log on the same goal.

---

#### DISP-1 — the DEGENERATE vertical toss at centre (**never flown; run it first**)

`B` = the live pose = centre, so displacement is `0 mm`, the aim is exactly level,
and `compute_release_state_tilted` returns the Tier-8a release state **bitwise**.
This is the operator's "8b subsumes 8a" expectation and it is the cheapest possible
first exercise of the new throw-site read.

```bash
# platform at centre: go_home first.
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-1.1 | node outcome line | not a `REJECTED_*` | `REJECTED_POSE_UNKNOWN` ⇒ re-run DISP-0.4. `REJECTED_DISPLACEMENT` ⇒ the platform was not where you thought; echo `/trajectory/commanded_position` and repeat |
| DISP-1.2 | commanded platform tilt over the whole goal (FK of `/leg_setpoint_echo`) | `\|rx\|`, `\|ry\|` within **`0.05°`** of the levelling target throughout the pre-throw phase | any pre-tilt at all — displacement is 0, so the aim must be exactly level |
| DISP-1.3 | deferred reach travel (commanded xy span from `t_release` to landing) | **`< 2 mm`** | `> 5 mm` — a zero-displacement reach is moving the platform |
| DISP-1.4 | caught, over 3 attempts | **≥ 2/3** | `0/3` ⇒ stop; this is the simplest throw the machine can make |
| DISP-1.5 | terminal | `Toss CAUGHT`, then **no `go_home`** — the platform stays where it is | a `go_home` after a CAUGHT ⇒ the install is stale (DISP-0.1) |

#### DISP-2 — the DEGENERATE vertical toss at an OBLIQUE pose (**never flown**)

The same zero-displacement throw, from `(140, −140)`. Under the retired config throw
site this goal was a **198 mm displaced** throw and would have refused; it is the
single clearest demonstration that `A` is now the live pose.

> **Why `(140, −140)` and not `(150, −150)`.** HISTORICAL, and kept because the
> pose is still the one to use: `150.0` was the *exact* value of the retired
> `TOSS_XY_LIMIT_MM`, and Phase E added a pre-check refusing a Tier-8b goal whose
> **throw site** `A` sat outside that box. Driving there by hand gives an
> arbitrary float — `(150.4, −149.7)` echoes as "~(150, −150)" and refused
> `REJECTED_WORKSPACE`. **That box was deleted 2026-08-29**, so `(150, −150)`
> would now be admitted; `(140, −140)` is retained anyway so this rung's numbers
> stay comparable with every previous run of it.

```bash
ros2 service call /trajectory/go_to_pose ...    # or drive there; then confirm:
ros2 topic echo /trajectory/commanded_position --once     # expect ~(140, -140, 170)
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 140.0, y: -140.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-2.1 | node outcome line | not a `REJECTED_*` | `REJECTED_DISPLACEMENT` ⇒ the throw site is not being read live — **stop**, this is the phase's headline defect. `REJECTED_WORKSPACE` ⇒ **a FINDING since 2026-08-29, not the old park refusal.** That code now covers the **z band alone** (`abs(B.z − 170) > 50`), and this rung's `z` is exactly `170.0`, so it cannot fire for a lateral reason at all — the `±150 mm` planning box it used to name is deleted. If you see it here, record the full outcome string (it quotes the offending `B.z` and `TOSS_Z_BAND_MM`) and route to `single-ball-toss` Phase E. The far-lateral refusal that DID move is now `REJECTED_POSITION(<planner code>: …)` at CHECKING on Tier 8b — see the AMENDED banner |
| DISP-2.2 | coordinator log | `toss declared catch reach centre (150.0, -150.0, 170.0) mm STOW` | a different centre, or no line at all |
| DISP-2.3 | POSITIONING move travel | **`< 5 mm`** — a pre-tilt in place, not a traverse | `> 20 mm` ⇒ the platform is being sent somewhere; the throw site read is wrong |
| DISP-2.4 | caught, over 3 attempts | **≥ 2/3** | `0/3` ⇒ record the arrival offset and stop. `(140, −140)` sits just inside the sitting's **worst extremity pose** for commanded-vs-mocap tilt, measured at `(150, −150)`: `0.604°` mocap residual, `42 mm` of implied lateral drift over a 0.78 m flight (`logbook/2026-07-28-anomaly-fixes-validation-sitting.md` § Extremity tilt). That residual is not modelled anywhere and is the likeliest cause of a failure here — more likely than anything Phase E changed |

#### DISP-3 — displaced **70 mm** (parity with the validated T4)

Identical geometry to § CHECK TIER-E, run here as the ladder's known-good anchor:
this exact displacement is **11/11 accepted with the deferred reach firing correctly**
on 2026-07-27. From centre, aim `−x`, `T ≥ 0.80 s`.

```bash
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: -70.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-3.1 | first `catch/dynamic_target` vs `t_release` | **`+0.000` to `+0.100 s`**, never negative (measured `+0.013…+0.050` on 11/11) | any value `< 0` |
| DISP-3.2 | deferred reach travel | **`50–75 mm`** (measured `51.5–72.2 mm`) | `< 50 mm` |
| DISP-3.3 | commanded pose span over `[release − 0.10 s, release]` | **`< 0.02°` and `< 0.2 mm`** | any commanded motion through release |
| DISP-3.4 | caught, over 3 attempts | **≥ 2/3** | `0/3` ⇒ **stop the ladder** — the known-good rung regressed, so nothing above it is interpretable |
| DISP-3.5 | ball arrival offset vs the nominated `B` (`ball_arrival_offset.py`) | record it; expect the same class as the 2026-07-27 self-tosses | *(no ABORT — this is the baseline the next two rungs are compared against)* |

#### DISP-4 — displaced **100 mm** (first rung past hardware evidence)

From centre. `T ≥ 0.80 s` — at `T = 0.55 s` the closed-form reach bound is `83.2 mm`
and the goal will be refused `REJECTED_DISPLACEMENT`, correctly.

```bash
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: -100.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-4.1 | goal accepted | yes | `REJECTED_DISPLACEMENT` ⇒ **not the cap — there is none since 2026-08-29.** At `T = 0.80 s` the closed-form reach bound is `256 mm`, so a `100 mm` goal cannot break it from centre. Two live causes: the platform was **not at centre** (the bound is on `|B − A|`, and `A` is the LIVE commanded pose — check `/trajectory/commanded_position`), or the **live `set_limits` are below the working point** (check `trajectory/status`; the bound follows them, so a ramped-down session shrinks it). Fix whichever it is and retry |
| DISP-4.2 | deferred reach travel | **`85–115 mm`** | outside — the reach is not spanning A→B |
| DISP-4.3 | DISP-3.1 and DISP-3.3 re-checked | same PASS bands | same ABORTs |
| DISP-4.4 | caught, over 5 attempts | **≥ 3/5** | `≤ 1/5` ⇒ stop, do not attempt DISP-5. `2/5` ⇒ record the arrival offsets and **decide with the operator** before climbing |
| DISP-4.5 | ball arrival offset vs `B`, worst of 5 | **`≤ 2×` the DISP-3.5 worst** | `> 3×` ⇒ the aim is degrading with displacement faster than the cup can absorb; report the trend and stop |

#### DISP-5 — displaced **150 mm** (the operator's ordered working point)

From centre, and **`T ≥ 0.80 s`**. The closed-form quintic reach bound refuses
150 mm below `T ≈ 0.669 s`, and the production planner is only `3/8` directions at
`T = 0.55 s` — so a short flight here is not a marginal choice, it is the wrong one.

```bash
# REPEAT THIS PAIR for each of the 5 attempts. The go_home is now OPTIONAL — the
# lateral box that made it mandatory was deleted 2026-08-29. Keep it if you want
# each attempt measured from centre (the displacements below assume A = (0,0));
# it no longer prevents a refusal.
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: -150.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-5.1 | goal accepted | yes | ANY `REJECTED_DISPLACEMENT` here is a finding since 2026-08-29: at `T = 0.80 s` the closed-form reach bound is 256 mm and a 150 mm goal is nowhere near it, so a refusal means the live `set_limits` are lower than the working point (check `trajectory/status`) or the bound itself has changed. Record it and route to `single-ball-toss` Phase E |
| DISP-5.2 | deferred reach travel | **`130–170 mm`** | outside |
| DISP-5.3 | peak commanded leg jerk during the reach (`/trajectory/diagnostics` `peak_leg_jerk`) | **`< 30000 mm/s³`**, and expect roughly **`9000–11000`** at `T = 0.80 s` — record it. (Do **not** expect `17578`: that is the closed form's **platform-space** figure, `60·d/T³`. `peak_leg_jerk` is a **leg-space** measurand and the production-faithful sim puts it at `9742` max / `9371` mean over the 150 mm ring at `T = 0.80` — `temp/reports/toss_8b_phaseE_seed0.json`, 80 trials, ~0.55× the platform-space number. The gate's own summary reports `jerk 11203` required across the whole sweep.) | at or above `30000` ⇒ the reach is riding the gate; raise the throw height (longer flight) rather than pushing on. A reading **near `17578`** is not a pass-with-margin story — it is ~1.8× the expected leg-space value and means the reach is spanning more than `B` |
| DISP-5.4 | DISP-3.1 and DISP-3.3 re-checked | same PASS bands | same ABORTs |
| DISP-5.5 | caught, over 5 attempts | **≥ 3/5** | `≤ 1/5` ⇒ 150 mm is not supportable at this direction/flight — record it and route to `single-ball-toss` Phase E. **Do not climb the ladder further on this direction**; since 2026-08-29 there is no cap to stop you, so the ladder discipline IS the bound |
| DISP-5.6 | **the held tilt after a caught toss** — commanded `rx/ry` (FK of `/leg_setpoint_echo`) once the goal terminates, and the ball's mocap position over the following 10 s | **REPORT.** Expect a held receive tilt of a few degrees (the closed form gives `≈3.6°` of arrival angle at 150 mm / `T = 0.70 s`) and the ball still within `35 mm` (`GEOM_HAND_RADIUS_MM`) of the cup axis | the ball leaves the cup while the platform holds the tilted pose ⇒ **set `toss_stay_at_pose_on_caught: false`, regenerate, rebuild, relaunch**, and record it. This is the one residual `STAY` introduces: the catch already holds this tilt through its settle with a ball in it, but never indefinitely |

> ### FORMER KNOWN LIMITATION (dissolved 2026-08-14) — chaining at the cap was
> ### refused while the planning box equalled the cap. It is now ADMITTED.
>
> **History (measured at box = cap = 150):** after a CAUGHT toss whose `B` was at
> (or within ~3 mm of) the `150 mm` cap, the next Toss and the next Reload would
> BOTH refuse — loud, pre-throw, remedied by one
> `ros2 service call /trajectory/go_home std_srvs/srv/Trigger`.
> **From 2026-08-14 the box was the config key `toss_workspace_xy_mm` (shipped
> 160 = cap + 10 > cap × 1.03), the parked centroid sat INSIDE it, and a cap-edge
> chain proceeded. From 2026-08-29 BOTH KEYS ARE DELETED** — the box and the cap
> alike — so this limitation cannot re-bind at all and the mechanism below is
> retained purely as the record of a real measured effect. The centroid-vs-cup
> divergence itself is unchanged (~2.07 % of displacement, ~3 mm at 150 mm); it
> is simply no longer measured against anything.
>
> **Why.** The catch deliberately parks the platform *centroid* slightly OUTSIDE
> `B` so the *cup* lands ON `B` (swing compensation: `centroid = landing −
> hand_catch_offset · platform_z`). Measured through the production chain at
> `B = (−150, 0, 170)`, `T = 0.80 s`: the **cup** ends at `(−150.00, 0)` — exactly
> `B` — but the **centroid** ends at `(−153.10, 0)`. `trajectory/commanded_position`
> publishes the *centroid*, so the next goal reads `A = −153.10`, and both
> surviving gates are applied to that value:
>
> - the `±150 mm` planning box on `A` ⇒ `REJECTED_WORKSPACE` for **any** `B`;
> - the `150 mm` cap on `|B − A|` ⇒ `REJECTED_DISPLACEMENT` for a `B` back at centre.
>
> The offset is `hand_catch_offset · sin(receive tilt)` = `64.78 · sin(2.73°)` =
> `3.10 mm` at `T = 0.80` (`4.05 mm` at `T = 0.70`), i.e. `2.07 %` of the
> displacement — so it crosses the box at `|B| ≈ 147 mm` on-axis, and crosses the
> cap on the return leg at any radius ≥ ~`147 mm` in any direction. **Below
> ~`146 mm` chaining works normally**, which is why DISP-6 chains at `100 mm`.
>
> **What this means for the ladder (updated 2026-08-29):** DISP-5 may run with or
> without a `go_home` between attempts — a refusal following a cap-magnitude catch
> is NOT expected, and ANY refusal of this class is now a finding to record,
> because there is no longer a knob whose setting could explain one. The
> centroid-vs-cup frame question itself stays open on `single-ball-toss` Phase E
> but gates nothing.

#### DISP-6 — the chained session A → B → C (**the point of the whole phase**)

Three tosses back to back with **no repositioning between them**. Each goal's `B` is
the next site; each throw's `A` is where the previous one caught.

**Chain at `100 mm` first, deliberately.** Historically a chained toss at
`150 mm` refused by construction (see the FORMER KNOWN LIMITATION box above);
since 2026-08-29 no lateral bound exists at all, so after the 100 mm chain
PASSES a 150 mm chain is a legitimate extension rung — record it as DISP-6x
rather than skipping it as impossible. Climb it deliberately: nothing in the
machine now refuses it for you.

```bash
ros2 service call /trajectory/go_home std_srvs/srv/Trigger
# A -> B
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 100.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
# B -> C   (no go_home, no reload if the ball was caught)
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 100.0, y: 100.0, z: 170.0}, throw_height_m: 0.78}" --feedback
# C -> back to centre
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.85}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-6.1 | each goal's declared centre in the coordinator log | matches that goal's nominated `B` | any mismatch |
| DISP-6.2 | goal 2 and goal 3 accepted **without** an intervening `go_home` or `Reload` | yes | `REJECTED_HAND_NOT_PARKED` ⇒ the catch left the hand off the park band (route to § SECTION POSS `POSS-2.1`). `REJECTED_DISPLACEMENT` here — **at `100 mm`, where the parked centroid is only `~102 mm`** — ⇒ the throw site did not follow the platform; **stop**, this is the chaining defect. (At a *cap-magnitude* `B` the same code is the KNOWN LIMITATION above, not a defect: check which one you are in by reading `/trajectory/commanded_position` — `> 150` on either axis means the limitation.) `REJECTED_WORKSPACE` ⇒ likewise the limitation, not a defect |
| DISP-6.3 | goal 2's displacement, from the log's declared centre and the previous catch | `≈100 mm` (`(100,0) → (100,100)`), **not** `141 mm` | `141 mm` ⇒ `A` reverted to the origin |
| DISP-6.4 | caught, per goal | record all three | *(no ABORT — a drop mid-chain is a reload away)* |
| DISP-6.5 | platform position between goals | never returns to centre | a `go_home` between goals ⇒ stale install |

#### DISP-7 — the RELOAD seam: refuse, loudly, from an oblique park

The reload's catch is hard-fixed at the workspace centre and it never pre-positions.
With the platform parked off centre it would arm an envelope centred off `(0,0)` and
reject the incoming BB ball **mid-flight**. It must refuse first.

```bash
# leave the platform parked from DISP-6 (or drive to ~(150, 0)), then:
ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload "{}" --feedback
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| DISP-7.1 | node outcome line | `Reload REJECTED_NOT_CENTERED` | **anything that lets BB throw** — if you see a `bb/throw_at_target` call from an off-centre park, **E-STOP the session**: a ball is about to be thrown at a platform that cannot reach it, and the rejection will arrive mid-flight |
| DISP-7.2 | hand motion during the refused goal | **`0.0000 rev`** span | any — the refusal must precede `ACTION_PRIME_HAND` |
| DISP-7.3 | after `go_home`, the same Reload | accepted, normal choreography | still `REJECTED_NOT_CENTERED` ⇒ **first confirm `/trajectory/status` is still arriving** (a dead `trajectory_node` mints this code too — its `streaming` observation is a sticky last-value, so a dead link reads as a geometry fault; see `reload_sequencer.py`'s NOT_CENTERED comment). If the link is live, compare `/trajectory/commanded_position` against `(0, 0)`: the tolerance is **`66.5 mm`**, not the `80 mm` envelope — `80 − 64.78·sin(12°)`, because the reload's own pre-tilt shifts the commanded centroid `13.47 mm` off the catch point and the gate must leave room for it |

---

### What the sim gate DID say, and what it does NOT cover

The Tier-8b sim gate was re-run at the current machine for this phase
(`python sim/toss_gate.py --tier 8b --trials-per-point 10 --seed 0`, run
2026-07-29, wall **1099.9 s**): **PASS** — binding 50 mm ring 9/9 points at
`core_clean ≥ 9/10`, and the production-in-the-loop invariants clean
(`feas_viol 0`, `pump_rejects 0`, **55 770 / 55 770** emitted knots pump-accepted
across all 350 gating trials, `accepted 350/350`). The catch rate per ring, all at
`T = 0.80 s`, 10 trials per direction × 8 directions:

| ring | core_clean | worst direction |
|---|---|---|
| 0 mm (centre) | 9/10 | — |
| 50 mm (**binding**) | 79/80 | 9/10 |
| 70 mm | 78/80 | 9/10 (`−x`, `SW`) |
| 100 mm | 78/80 | 9/10 (`−x`, `+x`) |
| **150 mm (the cap)** | **76/80** | **8/10 (`SW`)** |

**The 2026-07-25 directional map is refuted, not merely stale.** It flagged the
`+y`/NW hemisphere as failing at 70 mm; on the current machine `+y` is
**10/10 at 70 mm, 10/10 at 100, 9/10 at 150** and NW is **10/10 at every radius**.
That is why the cap ships as a **simple scalar with no direction-awareness** — and
why the one direction worth watching is now `SW`, the *opposite* hemisphere, at the
cap only. Take that as a watch-item for DISP-5, not as a prohibition.

**Five things the gate does not tell you:**

1. **The catch RATE at the cap is not gated.** The 150 mm ring is ADVISORY, because
   the ball-lands-in-cup verdict is dominated by a release-scatter magnitude that is
   still the Phase-5 T0 **placeholder** (1 %). Gating the shipped cap on an
   unmeasured number would make the cap an artefact of it. DISP-4.4/5.5 are the
   real evidence.
2. **The contact-DETACH asymmetry map is 1-bit-per-cell evidence, whatever its
   `n` column says.** Its cells read 0/4 or 4/4 almost everywhere — that is the
   documented low-fidelity contact model, not a signal. (Until a bug found and
   fixed on 2026-07-29 it was *literally* one trial replicated: every trial in a
   cell ran the same seed. Fixed; the re-run at seed 0, 4 trials/cell, now shows
   real variation — `+x` at 70 mm / `T = 0.60` reads **1/4**, and the landing
   errors have a spread instead of being identical. The map is still not what the
   cap rests on.) The gating column above is: kinematic release, 10 trials per
   direction, genuine 8/10–10/10 spread.
3. **The 2026-07-25 asymmetry map is stale vintage AND refuted** — measured on the
   pre-2026-07-26 moving-rim machine (seat rate 0.07, pre-C-CATCH-1 arrival). Its
   `+y`-hemisphere weakness at 70-100 mm describes a plant that no longer exists,
   and the current gating column contradicts it directly. Do not use it to pick a
   direction.
4. **No off-centre throw site is swept for the CATCH.** The gate sweeps `B` about a
   chosen `A`, which is geometrically faithful, but every rung above except DISP-2
   and DISP-6 throws from centre. The extremity-pose behaviour the sitting measured
   (up to `0.604°` of commanded-vs-mocap tilt residual at `(150, −150)`, worth
   `≈42 mm` of lateral drift over a 0.78 m flight) is **not modelled anywhere** and
   is the most likely cause of a DISP-2 or DISP-6 failure.
5. **The firmware time budget.** Flights below `0.7 s` are firmware-marginal
   (the kind-1 windup race) and the sim does not model it. Every rung above
   specifies `T ≥ 0.80 s` for that reason as much as for the reach bound.

### Not in this section

- **Raising the cap above 150 mm.** The production planner accepts the A→B reach out
  to `~225 mm` (8/8 directions, `T ≥ 0.70 s` — `tools/probes/displaced_reach_frontier.py`,
  2026-07-29), so kinematics is not the binding constraint; evidence is. Bring
  DISP-5 data first.
- **The hand's end-stop margin.** Unchanged by this phase, still owed by
  § CHECK HAND-7. Every rung above uses `throw_height_m ≤ 0.85`, inside the band
  that section flags.
- **Reload catching at a displaced site.** The reload's catch point is still
  hard-fixed at the workspace centre; DISP-7 refuses rather than extends it. Making
  reload displaceable is its own phase.
- **The tracker's split-track corruption on the reload path.** Separate open
  investigation; it is why DISP-7 scores a *refusal*, which needs no tracker.

---

## SECTION CONT — repeated toss-catch cycles (`single-ball-toss` Phase F)

> **⏩ WHERE TO GO NEXT (added 2026-08-22, corrected the same day).** This
> section validates the session at the SHIPPED 6.0 s dwell. Walking the dwell
> DOWN toward the tuning-phase operating point — **0.63 s at flight 0.5029 s,
> 53.0 throws/min on a level chain; 1.01 s, 39.7/min once an aim is armed** — is
> its own ladder with its own per-rung PASS/ABORT criteria:
> [`tests/hardware/session_cadence_ladder.md`](session_cadence_ladder.md), rungs
> R0 → R5-prime. **This section IS its R0.** Get SECTION CONT green here first;
> the ladder's every rung inherits the plant health it establishes.
>
> The **0.49 s / ~61 throws/min** this box named for half a day is not reachable
> on this build — the accept-time `throw_delay` floor models the kind-0 dispatch
> budget while the runtime guard measures it after the whole pre-dispatch
> sequence, so R4, R5 and R5-prime as first published aborted every cycle. Read
> the ladder's § 2.0 before booking any sitting below R3.

> **Appended 2026-07-29.** This is operator decision **(c)** of 2026-07-28 —
> *`toss_continuous {catch_position, throw_height_m, num_throws, dwell_time_s}`,
> `stop_on_miss` defaults **TRUE***. Validates `plans/active/single-ball-toss.md`
> **Phase F**. Run it **last**: it is the only section that repeats an actuation
> unattended, so it must come after § SECTION POSS (whose verdicts it consumes and
> which is how `stop_on_miss` knows anything at all) and after § SECTION DISP
> (whose STAY terminal is what makes chaining possible).
>
> **The action adds no new capability.** Every cycle is an ordinary `Toss`, built
> and ticked by the same code, with the same preconditions, the same arming order,
> the same abort ladder and the same terminals. What is new is *when* the next
> cycle starts and *whether* it starts at all. If a cycle behaves oddly, the fault
> routes to the section that owns that behaviour — the session's outcome string
> carries the cycle's own verdict verbatim (`ABORTED_CYCLE_REJECTED_HAND_NOT_PARKED`
> and so on) precisely so this routing is mechanical.

### Why no hand move happens between cycles — the fact to sanity-check by eye

The firmware catch stroke **ends at 0 rev** (`Trajectory.h` `buildCatch`,
`xA = {x3, x5, x6, 0.f}`) and a kind-0 throw stroke **starts at 0 rev**
(`buildThrow`, `xA = {0.f, x1, x2, x3}`). The catch is its own re-park; the throw
is its own catch-prime. Measured on the 2026-07-27 sitting: hand `pos_meas` is
within **±0.045 rev** of park at the CAUGHT instant on all 17 self-tosses, worst
excursion **0.069 rev** over the following 3 s against the ±0.5 rev park band
(**7.2×**). **Watch the hand between cycles: it must not move at all.** Any
between-cycle hand motion is a CS-5 finding and stops the section.

### Deployment for this section — the interfaces build is MANDATORY here

1. **`colcon build --packages-select jugglebot_interfaces jugglebot` + relaunch.**
   **Both packages.** This section adds a NEW action (`TossContinuous.action`), so
   the interfaces package genuinely must rebuild — and because
   `reload_coordinator_node` imports `TossContinuous` at module scope, that is now
   true of **every** section of this runbook, not just this one (the four
   per-section blocks that used to say "no interface rebuild" were corrected
   2026-07-29). A `jugglebot`-only build does not merely leave
   `/jugglebot/toss_continuous` missing: the node raises `ImportError` before
   construction and `Reload` and `Toss` disappear with it.
2. **No firmware flash for this section.** Config regeneration added three
   `constexpr` to the delivered `hardware_config.h`; no sketch reads them.
3. **⚠ GATE — `platform_fw_version = 2` before ANY multi-cycle run above 0.6 m.**
   The § CHECK HAND-7 decel-feedforward fix (`FW_VERSION` 1 → 2) is what keeps the
   hand off its end stop at height. A session multiplies exposure to that overshoot
   by `num_throws`, unattended. Read the version with the § Section FW command
   (`FW-1`); if it reads `0 (PRE-VERSIONING)` or `1`, this section is limited to
   `throw_height_m ≤ 0.60` and `num_throws ≤ 3`, full stop.

### Pre-flight CONT-0 — the installed copy really is Phase F (no robot, ~30 s)

```bash
INST=~/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -n "JB_OP_TOSS_SESSION_DWELL_DEFAULT_S\|JB_OP_TOSS_SESSION_DWELL_MARGIN_S\|JB_OP_TOSS_SESSION_MAX_THROWS" $INST/hardware_config.py
ls $INST/toss_session.py
# with the graph up:
ros2 action list | grep toss_continuous
ros2 interface show jugglebot_interfaces/action/TossContinuous | grep -n "stop_on_miss"
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| CONT-0.1 | the three new constants | `6.0`, `0.6`, `20` | missing ⇒ stale install; a different value is fine but **read it off this line and use it below**, do not use the printed defaults |
| CONT-0.2 | `toss_session.py` present in the install tree | yes | no ⇒ `jugglebot` not rebuilt |
| CONT-0.3 | `ros2 action list` | `/jugglebot/toss_continuous` present | absent ⇒ `jugglebot_interfaces` not rebuilt, or the launch was not restarted |
| CONT-0.4 | the goal field default | `bool stop_on_miss true` | anything else ⇒ **STOP.** The wire default is load-bearing: an omitted field must mean STOP, and a `false` default would run the whole session over a ball on the floor |

### CONT-STEP-0 — the zero-code dispatch-loop baseline (do this FIRST)

Before running a single `toss_continuous` goal, produce the comparison data by
hand: send **3 separate `Toss` goals** from a shell loop with a sleep between them,
exactly as a session would. This costs nothing, needs no new code path, and it is
the only way to tell a *session* defect from a *toss* defect afterwards.

```bash
# ONE ball, loaded once by a Reload; then three separate tosses, by hand.
# (Reload first — see the § Recording list for the recorder + bag commands.)
for i in 1 2 3; do
  ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
    "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.60,
      throw_delay_s: 5.0, catch_vel_scale: 0.0}"
  sleep 3
done
```

| # | quantity | PASS | REPORT / ABORT |
|---|---|---|---|
| CONT-B.1 | outcomes of the three hand-dispatched tosses | ≥ 2 of 3 `CAUGHT` | 0 or 1 ⇒ **ABORT the whole section.** The single toss is not healthy enough to repeat; go fix that first (§ SECTION POSS / § SECTION DISP) |
| CONT-B.2 | hand `pos_meas` between tosses (`--timeline` of the recorder capture, or the § CHECK HAND probe) | stays inside `±0.5 rev`; no commanded move between goals | any between-goal hand motion ⇒ **ABORT.** The no-hand-move premise is wrong on this machine and every rung below is invalid |
| CONT-B.3 | goal 2 and 3 accepted with **no** `REJECTED_TRACK_ACTIVE` | accepted | a track-active refusal ⇒ REPORT the delay you needed; the session's dwell must exceed it |

### CONT-STEP-1 — the DRY TRACE, no ball (mandatory before any live-ball session)

A 3-cycle session with an **empty cup**. ~~Because
`jugglebot_operational.toss_require_ball_evidence` is `false` (the operator
guarantees the ball; there is no ball-in-cup sensor), **every cycle fires a real,
empty throw stroke** — it does not refuse.~~ **That premise expired 2026-08-10 —
read the box immediately below before running this step.**

> **⚠ THIS STEP NEEDS THE ESCAPE HATCH SINCE 2026-08-10.** The gate now defaults
> `true` and reads the hand ball sensor live, so an empty-cup session is refused
> `REJECTED_NO_BALL` at cycle 1 and **the dry trace never fires a stroke** — the
> step cannot do its job as written. To run it, set
> `toss_require_ball_evidence: false` (config → `python config/generate_config.py`
> → `colcon build --packages-select jugglebot` → relaunch) for the dry trace only,
> and set it back before the live-ball session.
>
> The trace-only waiver parameter is the cheaper alternative and it **does** cover
> a whole session — `_build_toss_cycle` re-reads the parameter on every cycle, so
> all three cycles are waived and each logs its own WARN (verified in code
> 2026-08-10, not assumed). Prefer it: it needs no rebuild and no relaunch, and it
> cannot be left switched on by accident across a power cycle the way a config
> default can. Whichever you use, say which in the session notes. The rest of this
> step is unchanged. That is the same actuation § SECTION
TIER's Phase-3 dry capture already ran and is safe with the cup empty: the stroke
is a normal kind-0 stroke, nothing is caught, and each cycle ends `MISSED` through
its own `SAFE_ABORT` (retract, latch down, `go_home`). Reaching cycle 3 therefore
**requires `stop_on_miss: false`** — this is the one sanctioned use of that flag.

```bash
# recorder (system python3 + ROS env, NOT the venv) — see § Recording
python3 tests/hardware/toss_trace_recorder.py record --out temp/logs

ros2 action send_goal -f /jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.60,
    num_throws: 3, dwell_time_s: 8.0, throw_delay_s: 5.0,
    catch_vel_scale: 0.0, stop_on_miss: false}"

# then, under the VENV (standing rule 5):
python tests/hardware/toss_trace_recorder.py check \
  temp/logs/toss_trace_<stamp>.jsonl --continuous --timeline
```

> **The checker has been validated against THIS trace shape, offline, before you
> run it.** `tools/probes/toss_trace_synth.py`'s matrix carries two continuous
> happy cases, one per session terminal ladder: `cont_happy` (every cycle CAUGHT —
> the `ACTION_STAY` teardown) and **`cont_dry`** (every cycle MISSED — the
> `SAFE_ABORT` teardown, i.e. exactly what this step produces, including the ~10
> rev hand retract that lands inside every dwell window CS-3 measures). Both must
> read 6/6 PASS, exit 0. If you want to confirm the installed checker before
> trusting a capture from it, under the venv:
> `python tools/probes/toss_trace_synth.py --all --verify` — **30/30 OK, matrix
> CLEAN** is the expected line, and it needs no robot. Until 2026-07-29 only the
> all-CAUGHT case existed, and the checker as reviewed would have FAILed CONT-1.3
> on a correct capture of this step.

| # | invariant | what it would catch | PASS | ABORT |
|---|---|---|---|---|
| CONT-1.1 | **CS-1** cycle accounting | a session that silently restarted or skipped a cycle — the accounting you score the sitting from would be wrong | `PASS`, 3 cycles, one `TossContinuous` outcome line | any `FAIL` |
| CONT-1.2 | **CS-2** latch lifecycle | a **leaked armed latch across a dwell** — `catch_coordinator`'s reactive catch path live over a loaded cup for the whole gap, so any tracked ball can command platform motion | `PASS`, strict True/False alternation, ends disarmed | any `FAIL` ⇒ **STOP.** Safety-relevant |
| CONT-1.3 | **CS-3** dwell quiescence | session-level motion, or an auto-prime ascending with a ball in the cup | `PASS`: both dwell gaps silent on `dynamic_target`/`announce`/`vel_scale`/`prime_dispatched`, the hand command echo reaches the `±0.5 rev` park band in every gap, and never ASCENDS more than `0.15 rev` above the lowest value seen in that gap | any `FAIL` ⇒ **STOP**. Note what is deliberately NOT a failure here: on this MISSED capture each cycle's own `SAFE_ABORT` retract (up to `~10 rev`, downward) lands inside the measured gap by construction — the coordinator disarms BEFORE it retracts. CS-3 scores ascents only. The `0.15 rev` bound is measured, not chosen: worst real ascent over the 16 post-disarm gaps of `temp/logs/toss_trace_2026-07-27_15-39-50.jsonl` is `0.0440 rev` (3.4×), and an auto-prime ascent is `~9.96 rev` (66×) |
| CONT-1.4 | **CS-4** envelope re-declared | a stale `catch/reach_center` on cycle *k*, which puts the envelope back on the commanded pose and rejects the deferred A→B reach `WORKSPACE` **mid-flight** (the 4/4 hardware failure C-REACH-1 exists to close) | `PASS`, exactly one declaration per cycle, ≥ 10 ms before its arm | any `FAIL` |
| CONT-1.5 | **CS-5** no hand move | the firmware premise wrong on this machine ⇒ off-band kind-0 dispatch | `PASS`, hand inside `±0.5 rev` at every arm | any `FAIL` ⇒ **STOP**, and re-read CONT-B.2 |
| CONT-1.6 | **CS-6** release ordering | a clock-domain error between the FSM's perf clock and the announcement's ROS `throw_time` | `PASS`; no cycle releases before the previous scheduled landing. CS-6 does **not** score the achieved cadence against the REQUESTED dwell — the request is not in the trace, so a cadence that collapsed to a still-positive gap would PASS here. Row CONT-1.8 (and CONT-2.5 on the live rungs) is what scores the request; the achieved value CS-6 prints is the INDEPENDENT wire-side measurement of the same quantity, which is what makes the pair a cross-check rather than the coordinator marking its own homework | any `FAIL` |
| CONT-1.7 | session result | — | `outcome: COMPLETED`, `throws_completed: 3`, `catches_confirmed: 0`, `per_cycle_outcomes: [MISSED, MISSED, MISSED]`. **Reachable only with the escape hatch raised** (see the box at the head of this step): under the shipped `toss_require_ball_evidence: true` an empty cup gives `ABORTED_CYCLE_REJECTED_NO_BALL` at cycle 1 and none of CONT-1.1…1.8 gets its data | `catches_confirmed > 0` on an empty cup ⇒ a possession verdict is minting CAUGHT out of nothing; **STOP** and route to § SECTION POSS. `ABORTED_CYCLE_REJECTED_NO_BALL` ⇒ the hatch is not actually raised — that is the gate working, not a session fault |
| CONT-1.8 | `per_cycle_dwell_s` | — | entry 1 is `nan`; entries 2–3 are `≥ 8.0` | any entry **below** `8.0` ⇒ the same fault CS-6 catches, **STOP**. Values above `8.0` are EXPECTED here and are not a fault: a MISSED cycle the session continues past waits out a **2.60 s cleanup floor** from its scheduled landing (`CATCH_CONFIRM_WINDOW_S 0.56` + the `2.0 s` `go_home` profile + 2 node ticks at the post-2026-08-22 `_TICK_S` of 0.02), because `_safe_abort` dispatches the retract and the recentre on service ACKS and returns while both are still moving. At the `5.0 s` delay that floor puts the earliest release at `7.60 s`, i.e. **below** the `8.0 s` request, so the floor should NOT bind here and the entries should read close to `8.0`. **REPORT** anything above `8.60` with the value. (Both numbers moved 2026-08-21: `CATCH_CONFIRM_WINDOW_S` became DERIVED from `ball_possession.ARRIVAL_BAND_MAX_S` — the measured arrival-band ceiling, rounded up — rather than hand-written at 0.7, which sat 98 ms UNDER the band a sensor-primary verdict has to outlast. **Both moved again on 2026-08-24**, from that same one constant: the post-FW-14 re-measure collapsed the band to +87.6…+554.7 ms, so `ARRIVAL_BAND_MAX_S` 0.80 → 0.56, `CATCH_CONFIRM_WINDOW_S` 0.80 → 0.56 and this floor 2.84 → **2.60 s** — 240 ms off every missed cycle a session continues past. (this row read 2.90 before today — that value still carried the 0.05 s `_TICK_S` retired on 2026-08-22; at the shipped 0.02 s tick the pre-re-measure floor was 2.84.) `logbook/2026-08-24-arrival-band-remeasure.md`.) |

### THE LADDER — live ball, climb in order

`go_home` before every `Reload`, exactly as § SECTION DISP requires (a caught toss
STAYs, and the Reload precondition tolerance is `66.5 mm`).

| rung | goal | why here |
|---|---|---|
| **CONT-2** | `num_throws: 3`, `catch_position: (0, 0, 170)`, `throw_height_m: 0.60`, `dwell_time_s: 8.0`, defaults otherwise (`stop_on_miss` omitted ⇒ TRUE) | the first live session, at the height § CHECK HAND-7 has cleared without `FW_VERSION 2`, at centre where nothing else is in play |
| **CONT-3** | as CONT-2 but `num_throws: 5` | repetition without changing anything else |
| **CONT-4** | `num_throws: 5`, `throw_height_m: 0.80` | **only after `platform_fw_version = 2` is confirmed** (§ Section FW row FW-1). Unattended repetition is exactly the exposure the decel fix removes |
| **CONT-5** | `num_throws: 3`, `catch_position: (70, 0, 170)`, `throw_height_m: 0.80` | the chained displaced session — cycle 1 throws `0 → 70`, cycles 2+ are near-degenerate at `70` |

```bash
ros2 action send_goal -f /jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.60,
    num_throws: 3, dwell_time_s: 8.0, throw_delay_s: 5.0, catch_vel_scale: 0.0}"
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| CONT-2.1 | `outcome` | `COMPLETED` with `catches_confirmed == num_throws`, **or** `STOPPED_ON_MISS` (a real miss is data, not a fault) | `ABORTED_CYCLE_*` ⇒ read the embedded cycle verdict and route it to that verdict's own section. Do **not** re-run the session to "see if it works" |
| CONT-2.2 | **the stop actually stops** — on any `STOPPED_ON_MISS` | the machine performs **no further stroke** after the missed cycle's `SAFE_ABORT`, and `len(per_cycle_outcomes) == throws_completed` | another stroke after a miss ⇒ **E-STOP.** `stop_on_miss` is the safety path: a loose ball on the floor under a stroking machine is the whole reason it defaults TRUE |
| CONT-2.3 | eye-scored catches vs `catches_confirmed` | they agree (self-toss verdicts are trustworthy since C-POSSESS-1) | a disagreement is a **finding** — § SECTION POSS, not this section |
| CONT-2.4 | hand between cycles | visibly still; `per_cycle` shows no `REJECTED_HAND_NOT_PARKED` | any ⇒ **STOP**, CS-5 class |
| CONT-2.5 | `per_cycle_dwell_s[1:]` | every entry in `[8.00, 8.60]` — the request, plus at most the `0.6 s` handoff margin | below `8.00` ⇒ cadence inversion, **STOP**. Above `8.60` ⇒ **REPORT** with the value; something in the CAUGHT teardown is slower than the measured `0.442 s` verdict latency + 2 ticks |
| CONT-2.6 | `per_cycle_catch_error_mm` | all finite entries `< 40 mm`, and **not growing** across cycles | a monotonic growth across cycles ⇒ REPORT: the chained throw site is walking. Log `uptime_ms` with it (standing rule 4 — the Teensy-uptime lag) |
| CONT-2.7 | **the held tilt after a reload INTERLUDE** — the twin of DISP-5.6, for the other catch. Applies only on a rung run with `on_empty_cup: RELOAD`. Score commanded `rx/ry` (FK of `/leg_setpoint_echo`) once `reload interlude CAUGHT` is logged, plus the BB ball's mocap position over the whole hold until the resumed cycle's POSITIONING commands the next move | **REPORT.** Since 2026-08-29 the interlude's CAUGHT terminal is `_recenter_stay` — RECENTER **minus** `go_home` — so the platform HOLDS the reload catch's receive tilt with the ball in the cup. Expect it **AT THE 12° CEILING**, not a few degrees: real BB arrivals are 18–40° off vertical and `compute_catch_orientation` clamps at `MAX_TILT_DEG`, so the tilt saturates on every arrival and the cup shifts an invariant `64.78·sin(12°) = 13.47 mm` laterally (see `_RELOAD_CENTERED_TOL_MM`'s comment). PASS = the ball stays within **35 mm** (`GEOM_HAND_RADIUS_MM`) of the cup axis for the whole hold — 2.6× of margin on the worst case | the ball leaves the cup while the platform holds the tilted pose ⇒ **ESCAPE HATCH: revert the interlude to the pre-2026-08-29 recentre** by passing `stay_on_caught=False` (i.e. dropping the `stay_on_caught=True` argument) at the interlude's `_step_sequence` dispatch in `reload_coordinator_node.py` — one keyword, `_recenter` is byte-identical to what shipped before — then rebuild, relaunch and record it. The standalone `jugglebot/reload` action is unaffected either way. Cost of reverting: the 2.80 s serial recentre the fix removed comes back |
| CONT-3.1 … CONT-5.1 | as CONT-2.1 … CONT-2.6, per rung | — | — |

### Refusals you should expect, and what they mean

| outcome | meaning | what to do |
|---|---|---|
| `REJECTED_NUM_THROWS` | `num_throws` outside `[1, 20]` | fix the goal |
| `REJECTED_THROW_DELAY` | `throw_delay_s` below the toss FSM's own delay gates, mirrored at session scope so the verdict names the field that is wrong. **The flat `MIN_TOSS_THROW_DELAY_S = 3.5 s` retired 2026-08-22** (census A1) and the replacement gained its second term on 2026-08-23: both gates now import one derivation, `toss_sequencer.min_throw_delay_for_release_s` = `max(TOSS_DISPATCH_DEBOUNCE_S = 0.10, hand_stroke.min_throw_event_delay_s(v_throw) + pre_dispatch_budget_s(...))` — the Teensy's own `:642` dispatch budget (**0.337 s** at the 0.4949 s band floor, **0.281 s** at the 0.80 s nominal) PLUS the pre-dispatch sequence the runtime guard measures it after (**0.080 s** when POSITIONING takes the census-B1 skip, **0.460 s** when it commands the move). Charging the dispatch budget alone let a goal be ACCEPTED and then abort `ABORTED_CANT_MAKE_RELEASE` on every cycle. Without the mirror a goal with an illegal delay satisfies `dwell ≥ delay + margin`, is ACCEPTED, builds a whole cycle's per-goal state, and only then dies `ABORTED_CYCLE_REJECTED_CANT_MAKE_LEAD` | raise `throw_delay_s` above the floor for **your flight time** (or omit it for the `5.0 s` default). Nothing moved |
| `REJECTED_DWELL` | the dwell is below `max(throw_delay_s + handoff_margin_s, hand_floor_dwell_s)`. **Derived, not chosen**, and since 2026-08-22 it has TWO terms (census A3/A4). *Handoff:* a cycle's release is its own accept + `throw_delay`, and the session cannot start cycle N+1 before cycle N's possession verdict can EXIST — but `handoff_margin_s` is itself a `max(dwell_margin_s, catch_park_reentry_s)`, and **the park term is the one that binds**. `dwell_margin_s` tracks `ball_possession.ARRIVAL_BAND_MIN_S`, the earliest sensor `empty→held` edge ever observed: `+137 ms` until 2026-08-24, `+87 ms` since the post-FW-14 re-measure (it was `0.6 s`, sized on the *mocap tracker's* verdict, until possession went sensor-PRIMARY). The hand's park re-entry is **0.1416 s** at the R0–R3 flight on the binding aim-armed column — already above the OLD 0.137 s — so the 2026-08-24 re-measure moved this floor by **nothing at all**. *Hand geometry:* catch tail + prelude + gap + throw windup, in series, because C-HAND-1 forbids overlapping them — **`0.4871 s` at the 0.4949 s band floor**, and below ~0.5 s it is the ONLY term that binds. Floor is **`5.1416 s`** at the `5.0 s` default delay (`5.1204 s` with the aim disarmed) | raise `dwell_time_s`, or lower `throw_delay_s` toward its own floor. Do **not** treat the refusal as a defect: a cadence the machine quietly ignored would be a lie about what it did — and a dwell under the *hand* term does not merely run slow, it dispatches the next throw inside the live catch stroke |
| `REJECTED_CHAIN_UNREACHABLE` | **RETIRED 2026-08-29 — this code no longer exists.** It fired when a chained session's predicted cycle-2 throw site fell outside the `toss_workspace_xy_mm` planning box; that box was deleted, taking the cycle-2 refusal the gate existed to pre-empt with it. Historical frontier, at box = cap = 150: `\|B\| ≤ 146.5 mm` chained, `\|B\| ≥ 147.0` did not | seeing this string at all means the install predates 2026-08-29 — rebuild and relaunch |
| `ABORTED_CYCLE_<verdict>` | a cycle was REJECTED or ABORTED — a machine fault, not a missed catch. The session stops **regardless of `stop_on_miss`**, because repeating a fault `num_throws` times is how one fault becomes N | route the embedded verdict to its own section |
| `REJECTED_BUSY` | a Reload/Toss/session was already running | one ball-op at a time, across all three actions |

### What this section does NOT cover

- **A per-cycle waypoint list** (a different `B` each cycle, A → B → C). Explicitly
  out of scope for v1: it needs the throw-site frame question of § SECTION DISP's
  open question settled first, and a fixed `B` is the well-conditioned case
  (measured, a fixed-`B` chain **converges**).
- **Possession RETENTION across the dwell.** Nothing re-verifies that the ball is
  still in the cup between catch and next throw. C-POSSESS-1's verdict is minted at
  **arrival** — the tracker declares CAUGHT because the marker vanished — so a
  post-CAUGHT bounce-out during a dwell leaves the latch set and the next cycle
  fires an empty stroke (benign, but the verdict is wrong), and `stop_on_miss` does
  **not** close it because the bounce-out happens *after* a CAUGHT. A session
  multiplies that exposure by `num_throws`. **Watch the cup between cycles by eye,
  and score a lost ball as a finding even when the outcome line says CAUGHT.** The
  ball-in-cup hand sensor (installed 2026-07-28) closes this with no wire change —
  it becomes the primary source behind the existing `_possession_confirmed` seam.
- **Sessions above 5 cycles.** The config ceiling is 20; the evidence stops where
  this ladder stops.
- **Unattended operation.** Every rung above is watched. Nothing in this phase makes
  the machine safe to leave.

---

## SECTION SEAT-EXP — the seat-rate A/B (a SEPARATE, LATER sitting)

> **This is not part of § THE RUN SHEET.** That run sheet is marked EXECUTED
> 2026-07-27 and describes a sitting that has happened. SEAT-EXP is a fresh
> sitting with its own ordered stages below. Standing rules 1–6 still apply in
> full; nothing here replaces them.

### The question, and why the last sitting could not answer it

`ZSEAT-2` ABORTed on its bounce-out arm: three eye-confirmed bounce-outs off a
**stationary 10.8° reload rim**, all three consecutive, at the top of the
session. Its rate arm passed (13 caught / 16 = 0.8125) and its flatness arm
proved the zero seat really shipped, so the experiment measured what it set out
to measure. What it could **not** do is decide *why*, because the same capture
carries a second measured cause: a monotonic **26–39 mm Ball-Butler warm-up
drift** in arrival `x` that plateaued around the sixth throw. The three drops
arrived that much further `+x` than the thirteen catches, and the operator
independently eye-witnessed the throws being "visibly off" during exactly those
attempts.

The two hypotheses are **not alternatives**. A ~30 mm off-centre arrival on a
stationary tilted rim is a *description of the disturbance the 0.07 rad/s seat
existed to reject*, not a competing explanation for the drop. So the open
question is narrow and precise:

> **Does a non-zero seat rate widen the capture basin — does it catch a ball
> ~30 mm off-centre that the zero seat drops?**

Nothing in the 2026-07-27 capture can answer it, because that capture contains
**no throws at a large `+x` offset with a non-zero seat**. Full evidence:
[`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`](../../logbook/2026-07-28-anomaly-fixes-validation-sitting.md).

### THE ONE THING THAT DECIDES WHETHER THIS SITTING IS WORTH RUNNING

**Both arms must be flown in the marginal regime.** This is a design constraint,
not a preference, and it is derived from the last sitting's own numbers.

At the *plateau* offset (`x@1000 ≈ −27 … −54 mm`) the zero seat caught **every
attempt that got there — 10 of 10 scored**. (The sitting as a whole was 13 caught
of 16 attempted. The bookkeeping: 18 announcements, 2 of which aborted before a
ball left the Butler, leaves 16 attempts; 14 of those have a scoreable track. Of
the 14, three dropped — all in the marginal band, at `+2.8 / −8.9 / −11.5` — one
caught in the marginal band at `−11.5`, and the remaining 10 caught out on the
plateau. The last 2 catches are the real balls among the four `NO TRACK`
announcements, so their arrival offset is simply unknown.) Running the A/B at the
plateau means the `0.0` arm bounces out ~0 times, the
`0.07` arm bounces out ~0 times, and the sitting returns `0 vs 0` — which is
**INCONCLUSIVE with certainty**, and costs a full sitting to learn nothing.

The measured basin edge sits at **`x@1000 ≈ −12 mm`**: attempts 1–3 dropped at
`+2.8 / −8.9 / −11.5`, attempt 4 caught at `−11.5`, and every attempt from
`−27.3` outwards caught. **That edge is degenerate** — the drop and the catch
that bracket it arrived at the same offset to within 0.02 mm — which is why no
absolute number from the old hand analysis is a gate anywhere in this section and
why the decision rule is strictly within-sitting.

So the marginal band is **two-sided**:

    −15 mm  ≤  mean x@1000  ≤  −5 mm      (about +25 to +35 mm of aim bias off the plateau)

**The ceiling is not decoration.** A floor-only gate passes in both directions
that waste the sitting. Under-bias to `−14.5 mm`: with the measured 9–11 mm
throw-to-throw sd against a `−12 mm` edge, only ~38 % of throws land in the drop
zone, so the expected `k0` is ≈ 4.6 of 12 — *below* the `k0 ≥ 5` the decision
table needs, and `5 vs 1` is explicitly not decidable. Over-bias to `+10 mm`
(the aim mechanism is unspecified and the required shift is 25–35 mm, so
overshoot is a real risk): ~98 % of throws drop, `k0 ≈ 12` and `k1 ≈ 10–12`, and
nothing is decidable either. Aim for the **middle** of the band, near `−10 mm`
— which is where the last sitting's four marginal attempts actually sat (mean
`−7.3 mm`, `p0 ≈ 3/4`), and where the power table's assumptions hold.

**How to move the arrival offset is the operator's call and is NOT prescribed
here** — it is Ball-Butler aim, and this file does not invent a command it has
not verified. What is prescribed is that you **measure** it live (row
`SEAT-EXP-2`) and do not proceed to the scored blocks until the burn-in bag shows
the arrivals inside that band. If the offset cannot be moved into it, **stop and
record INCONCLUSIVE-BY-DESIGN**; do not run the blocks and do not report the
result as "the seat makes no difference".

### Deployment — a YAML key now, not a source edit

The seat rate is `trajectory_op.catch_seat_rate_radps` in
`config/hardware_config.yaml` (added 2026-07-28). It ships at **`0.0`** and the
default is unchanged by this experiment. The whole point of the key is that this
A/B is a config toggle: nothing is edited in `planner.py`, so the experiment
cannot leave an accidental permanent code change behind, and the deployed value
is readable out of the *installed* tree.

Flipping it requires **all four** steps. Skipping the middle two is silent:

```bash
# 1. edit config/hardware_config.yaml -> trajectory_op.catch_seat_rate_radps: 0.07
# 2. regenerate (venv — standing rule 5)
source ~/Desktop/PDJ_venv/venv/bin/activate
python config/generate_config.py
# 3. rebuild (NO venv — colcon builds against system python 3.8)
cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && cd ..
# 4. RELAUNCH jugglebot_launch.py — the launch runs the INSTALLED copy.
```

**No firmware flash.** The `jugglebot_interfaces` half of the build changes nothing
for this section, but it is mandatory in every section since 2026-07-29
(`reload_coordinator_node` imports `TossContinuous` at module scope — matrix
row B).

**Expected commanded-motion change on the reload catch**, measured 2026-07-28
through the production planner at the recorded reload geometry (lead 2.3712 s,
receive tilt 10.87°) — so an operator knows what "working" looks like:

| | seat `0.0` | seat `0.07` |
|---|---|---|
| segments / plan duration | 2 / 2.8712 s | 3 / 3.0212 s |
| arrival tilt rate at contact | `0.000000 rad/s` | `0.070000 rad/s` |
| settle `rx` / `ry` | `+1.774062 / −10.636334°` | `+1.844635 / −10.928741°` |
| predicted leg vel / acc / jerk | `29.0 / 37.9 / 170` | `23.8 / 142.0 / 3935` |
| vs session limits `1000 / 5000 / 30000` | 2.9 % / 0.8 % / 0.6 % | 2.4 % / 2.8 % / 13.1 % |

C-CATCH-1's bound at this geometry is **`0.200047 rad/s`**, so `0.07` is
returned intact and nothing is clipped. Note the leg **velocity falls** while acc
and jerk rise — that is arithmetic (a terminal rate along the travel lets the
reach coast slower through its middle), not a fault.

### Stage order

| stage | what | rate | scored? |
|---|---|---|---|
| 0 | prerequisites + `SEAT-EXP-1` deployment check | `0.0` | no |
| 1 | **BB burn-in**, **≥ 8** reloads, robot catching normally | `0.0` | **no — discarded** |
| 2 | `SEAT-EXP-2`: confirm the arrival offset has plateaued **and** sits in the marginal band | `0.0` | gate |
| 3 | **CONTROL block** — throw until **12 are SCORED** (expect to need ~14–15) | `0.0` | **yes** |
| 4 | flip to `0.07` (4 steps above) + `SEAT-EXP-3` deployment check | `0.07` | no |
| 5 | **EXPERIMENT block** — throw until **12 are SCORED** (expect to need ~14–15) | `0.07` | **yes** |
| 6 | `SEAT-EXP-8` revert to `0.0` + rebuild + relaunch | `0.0` | mandatory |

> **Throw more than 12 per arm, and score each bag before ending its block.**
> `SEAT-EXP-5` wants **12 *scored*** attempts, with missed arrivals and `NO TRACK`
> announcements excluded from the denominator — so throwing exactly 12 cannot
> satisfy it if even one ball misses the cup or loses its track. That is not a
> remote risk here: this protocol deliberately flies the arms **in the marginal
> band**, where a genuine missed arrival is materially more likely than at the
> plateau, and the reference capture already lost 4 of 18 announcements to
> `NO TRACK` for unrelated reasons. Run the probe on the block's bag *before*
> moving on, and keep throwing until it prints `n = 12`.

**Why a fresh control block when 2026-07-27 already ran 16 at `0.0`.** Because
the last sitting's `0.0` data is confounded by the very drift under test: its
three drops are exactly its un-warmed-up throws. A control block taken *after*
the burn-in, at the same offsets as the experiment block, on the same day, with
the same BB temperature, is the only `0.0` arm that is comparable to a `0.07` arm
run minutes later. Re-using the old 16 would re-import the confound the whole
experiment exists to remove.

### Prerequisites (stage 0)

- Standing rule 1 (as amended 2026-08-15): the can-bridge power-cycle is
  **retired**; log `uptime_ms` beside every timing number.
- Standing rule 2: check `gravity_correction_loaded`; `level` only if `false`.
- Standing rule 3: **one truthful outcome line per attempt, by eye.** In this
  section that is not a nicety — the operator's per-ball verdict is the
  **primary** outcome (see § Scoring). Reload `outcome` still reads `MISSED` on
  real catches and cannot be used as the numerator.
- Recording: rosbag on all topics, plus the trace recorder, for both scored
  blocks. `/mocap_data` is what the arrival-offset probe reads, so a bag without
  it cannot be scored at all.

#### CHECK SEAT-EXP-1 — the tree is at the shipped `0.0` before you start

```bash
INST=ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -n "JB_TRAJ_CATCH_SEAT_RATE_RADPS" $INST/hardware_config.py
grep -n "catch_seat_rate_radps" config/hardware_config.yaml
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/catch_reach_replay.py --self-check ; echo "exit=$?"
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| SEAT-EXP-1.1 | installed `JB_TRAJ_CATCH_SEAT_RATE_RADPS` | `= 0.0` | anything else — a previous experiment was never reverted; **stop and revert before capturing anything** |
| SEAT-EXP-1.2 | YAML `catch_seat_rate_radps` | `0.0` | disagrees with 1.1 ⇒ the install is stale; rebuild |
| SEAT-EXP-1.3 | `catch_reach_replay.py --self-check` | `SELF-CHECK: PASS`, `exit=0`, **no `BAD` lines** | any `BAD`, or `exit=1` |

> **Do not pipe the self-check through `tail -1`.** An earlier draft of this block
> did, and it defeats the row: `$?` then reports `tail`'s status, which is `0`
> unconditionally, so the `exit 0` half of the criterion becomes unobservable —
> and the `BAD` lines the row actually gates on are exactly what `tail -1` hides.
> Neither self-check prints an `N/N` tally, so "no `BAD` lines" is the criterion,
> not a count.

#### CHECK SEAT-EXP-2 — burn-in gate (stage 2). **This is the go/no-go.**

Score the burn-in bag *before* starting the control block:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/<BURN_IN_BAG>
```

| # | quantity | PASS | ABORT / STOP |
|---|---|---|---|
| SEAT-EXP-2.1 | arrival `x@1000` has **plateaued** — the probe's `BURN-IN TREND` line, fitted over the last **8** scored throws | **\|slope\| ≤ 3.00 mm/throw** | still walking ⇒ throw ~6 more and re-score. Measured on 2026-07-27: warm-up throws 1–6 read **−7.13 mm/throw**, the settled plateau **−1.75** (n=8), so the two regimes are cleanly separated by this threshold |
| SEAT-EXP-2.2 | the plateau sits in the **marginal band** — read the probe's **`PLATEAU (last 8 scored)`** line, *not* the all-scored summary line above it | **−15 mm ≤ mean `x@1000` ≤ −5 mm** | outside it *on either side* ⇒ **do not run the blocks.** Adjust BB aim and re-burn-in; if the offset cannot be moved there, record **INCONCLUSIVE-BY-DESIGN** and end the sitting |
| SEAT-EXP-2.3 | tracks are usable | ≥ 6 of the last 8 throws scored, **at most 1** flagged `SPARSE` | mostly `NO TRACK` / `SPARSE` ⇒ mocap coverage is too thin to score this experiment at all; fix that first. (2026-07-27 ran 14 usable of 18, with 1 `SPARSE`, so "none sparse" would have been a criterion that fires on a healthy capture) |

> **Why a slope and not "successive change < 10 mm".** The sitting's own
> recommendation phrased this gate as *successive-throw change under 10 mm*, and
> that criterion **fires on correct behaviour**: on the 2026-07-27 plateau —
> a demonstrably settled Butler — successive changes run
> `0.3, 15.6, 0.8, 11.5, 10.0, 9.0, 6.3` mm, three of seven over the line. The
> throw-to-throw scatter is ~9–11 mm sd, so a plateau is the absence of a
> *trend*, not of scatter. Both halves are pinned in the probe's `--self-check`
> (cases 7 and 8) so the rejected criterion cannot be re-adopted from the prose.
>
> **Read the PLATEAU line, not the summary line.** The probe prints two means:
> an all-scored one (`N scored past the warm-up; x mean …`) and a
> `PLATEAU (last 8 scored)` one. `SEAT-EXP-2.2` is a statement about the
> *plateau*, and the all-scored mean is not it — it still carries the warm-up
> excursion, which points toward `+x`, i.e. **toward passing the gate**. On the
> 2026-07-27 reference capture the two read `−28.8 mm` and `−38.9 mm`: a
> **10.1 mm optimistic bias**, two thirds of the whole margin between the band
> edge (`−15`) and the basin edge (`−12`). A Butler that had genuinely settled
> at `−22 mm` — outside the band, where the zero seat catches everything — would
> print ≈ `−11 mm` on the summary line, pass the go/no-go, and spend the sitting
> collecting `0 vs 0`. The `PLATEAU` line was added on 2026-07-28 for exactly
> this row.
>
> **This gate is a pre-filter, not the protection.** At the measured 11.4 mm sd
> the slope's own standard error is **2.72 mm/throw at n=6** and **1.76 at n=8**
> — so a residual drift of a few mm/throw is invisible to it, and over a 12-throw
> block that is tens of mm of separation between the arms. `SEAT-EXP-6` is what
> actually catches that, after the fact, from the two block means.

#### CHECK SEAT-EXP-3 — the flip actually deployed (stage 4)

Run **after** the rebuild and relaunch, **before** the experiment block. Four
independent reads — two static, two from the machine's own commanded motion —
because a silently-stale install is the failure mode that would make the whole
sitting compare `0.0` against `0.0` and report it as "no difference":

```bash
INST=ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot
grep -n "JB_TRAJ_CATCH_SEAT_RATE_RADPS" $INST/hardware_config.py     # expect 0.07
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/catch_reach_replay.py --self-check ; echo "exit=$?"
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| SEAT-EXP-3.0 | **the relaunch's own preconditions** — `ros2 topic echo /trajectory/status --once`, read `gravity_correction_loaded`; also re-record the can-bridge `uptime_ms` for this block | `true` | `false` ⇒ `level` from IDLE before throwing anything. **Stage 4 mandates a relaunch and a relaunch re-runs the `/gravity_offset` race** (standing rule 2): that topic is VOLATILE, so a `trajectory_node` that finishes subscribing after the push misses it. The toss path refuses loudly on this, but **the reload catch path has no such gate** — `platform_levelled` is consumed only by `toss_sequencer`, so 12 EXPERIMENT reloads would proceed silently with the platform commanded up to 0.78° off gravity. That is a difference in *rim orientation at contact* — the exact variable under test — and `SEAT-EXP-6` cannot see it, because it matches on the BALL's arrival offset, not the platform's attitude |
| SEAT-EXP-3.1 | installed `JB_TRAJ_CATCH_SEAT_RATE_RADPS` | `= 0.07` | still `0.0` ⇒ the `colcon build` was skipped or failed. **The launch runs the installed copy**; without this the experiment block silently repeats the control |
| SEAT-EXP-3.2 | `catch_reach_replay.py --self-check` | **`SELF-CHECK: FAIL (1 case(s))`, exit 1, and the single `BAD` line reads exactly `planner._CATCH_TILT_THROUGH_RATE_RADPS: production 0.07 != mirrored 0.0`** (9 of 10 cases `OK`) | any *other* `BAD` row, or `PASS`. **`PASS` here is the ABORT**: it means the tree is still at `0.0` |
| SEAT-EXP-3.3 | `peak_leg_acc_mmps2` / `peak_leg_jerk_mmps3` on a **reload catch install**, from `/trajectory/diagnostics` **in the first experiment-block bag** | `≈ 142` / `≈ 3935` | still `≈ 38` / `≈ 170` ⇒ the relaunch did not pick up the rebuild. **`0.0 / 0.0` is NEITHER** — see the note below; re-read, do not score it. **These are the same two numbers § CHECK ZSEAT-2 uses, read the other way round** — there they are the PASS for the zero seat, here they are the PASS for the flip |
| SEAT-EXP-3.4 | **REPORT ONLY, not a gate.** Commanded tilt over the last 0.8 s before landing (FK of `/leg_setpoint_echo`) — same invocation and the same `--t0/--t1` origin caveat as § CHECK ZSEAT-2's flatness row: `levelling_tilt_bag_check.py --t0 <landing−0.8> --t1 <landing> --plateau-min-s 0.1 --plateau-tol 1.0`, read `span_deg` | expect roughly **`0.9°`** of round trip — the rim is moving through the seat. Record whatever you get | — (do not abort on this row; `3.1` and `3.2` are the decisive reads) |

> **Not a segment count.** An earlier draft of `SEAT-EXP-3.3` asked for "3
> segments vs 2" — which the 2026-07-27 sitting had already recorded as **not
> scorable from a bag** (`CCATCH-2` row 4: all observables consistent with 2, the
> count itself unreadable). The two rows above are the same distinction expressed
> in quantities a bag actually carries.
>
> **`SEAT-EXP-3.3` must be read from the BAG, and most messages on that topic are
> zeros.** Do not reach for `ros2 topic echo /trajectory/diagnostics --once` (the
> idiom this file uses elsewhere): of the 209 distinct `move_seq` values on that
> topic during the 2026-07-27 sitting, **exactly 18 carried a non-zero
> `peak_leg_vel_mmps` — the other 191 read `0.0`**, because report-less installs
> publish zeros. A `--once` echo has a ~91 % chance of landing on one of those and
> returning `0.0 / 0.0`, which matches neither the PASS nor the ABORT. Read the
> topic **out of the experiment-block bag** and take the last message with
> `peak_leg_vel_mmps > 0` before the first ball arrival — that is the reload
> pre-tilt reach, which is the install this row is about. This file does not yet
> carry a verified one-liner for that extraction; the 2026-07-27 sitting did it
> offline, and inventing an unverified command here is precisely how
> `§ CHECK LVL-4` and `FW-1` broke.
>
> **Why `SEAT-EXP-3.4` is REPORT and not a gate.** Its `≈ 0.9°` is *derived*, not
> measured: nobody has yet run hardware at `0.07`. The value follows from the
> pre-tilt republish path — `_republish_pretilt` re-asserts on every `/balls`
> tick, the reach-freeze releases at `arrival + catch_settle_hold_s`, and each
> post-release republish installs a degenerate (seed == target) reach that at
> `0.07` manufactures ~`0.79°` of wrong-side excursion plus ~`0.30°` of settle
> overshoot inside the window. That reasoning is sound but unvalidated on
> hardware, and an offline single-plan rebuild that ignores the republish gives a
> materially different span (2–4°), so the two models disagree about the *window*
> even though they agree the tilt stops being flat. The zero-seat side IS measured
> (`span_deg = 0.0000`). Gating a deployment check on the derived half of that
> pair risks failing a correct deployment, so this row records and does not abort.

> **`SEAT-EXP-3.2` is deliberately an inverted row and is the only one in this
> file.** The probe's case 7 mirrors the shipped `0.0`, so a self-check FAIL
> naming that one constant is *positive confirmation the flip deployed*. It is
> also why the flip must be reverted before anything else in this file is run:
> row `INST-1` treats any `BAD` as an ABORT, and it is right to.

> **Expected red tests while the tree is flipped.** `pytest` is not part of a
> sitting, but if you run it: **exactly 6** tests pin the shipped zero seat and
> go red at `0.07` — `test_self_check_passes`,
> `test_self_check_catches_a_moved_production_constant`,
> `test_catch_reaches_pose_and_ends_at_rest`,
> `test_the_shipped_catch_is_reach_plus_quiescent_hold`,
> `test_the_shipped_default_manufactures_nothing_but_still_obeys_a_caller`,
> `test_dynamic_target_tilted_catch_is_stationary_at_the_shipped_default`
> (measured 2026-07-28 over the nine catch-touching test files: 251 passed,
> 6 failed; the same set reads 257 passed at `0.0`). That is the tree telling the
> truth about a deliberate temporary state. **All six must be green again after
> `SEAT-EXP-8`** — that is the revert's own check.

### Scoring

Three independent readings per attempt. They are ranked, and the ranking matters:

1. **The operator's eye — PRIMARY.** `caught` / `bounced out` / `missed arrival`
   for every ball, written down at the time. The 2026-07-27 sitting had to
   reconstruct this from traces afterwards and two of three classifications rested
   on inference; the testimony that arrived a day later is what settled them.
   Do not re-run that. **A bounce-out and a missed arrival are different events**
   and must be recorded separately: a bounce-out is evidence about the rim, a
   missed arrival is evidence about BB scatter and is *excluded from the
   numerator*.
2. **Arrival offset — the matched-arrival half.** Both blocks, same command:

   **Record ONE BAG PER ARM.** This is a hard requirement, not a convenience,
   and it is the difference between a scorable sitting and a silently mixed one.

   ```bash
   source ~/Desktop/PDJ_venv/venv/bin/activate
   # one invocation per arm, each bag scored on its own
   python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/<CONTROL_BAG> --csv
   python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/<EXPERIMENT_BAG> --csv
   # then, to get the MATCHED-ARRIVAL CHECK and the pooled sd that SEAT-EXP-6
   # and 7.1 read, score both arms in ONE invocation over a combined capture:
   python tools/probes/ball_arrival_offset.py --bag ~/Desktop/rosbags/<BAG> \
       --discard 8 --group CONTROL=9-20 --group EXPERIMENT=21-32 --csv
   ```

   > **Why per-arm bags, and why the `--group` ranges above are a trap if you
   > copy them blind.** `--group` selects on the probe's **attempt index**, and
   > that index counts `/throw_announcements` messages — *every* announcement,
   > including ones that never produce a scoreable ball. On the 2026-07-27
   > reference capture **4 of 18 announcements yielded `NO TRACK`**, and the
   > sitting independently records two Ball-Butler throws aborting
   > (`THROW_ABORTED_NOT_SETTLED`, `PV_STALE`) so no ball ever left. Both are
   > currently-open, recurring BB failures. So "the 12 reloads I threw in the
   > control block" and "attempts 9–20" are **not the same set**: two BB refusals
   > during CONTROL push its last two real balls to indices 21–22, and the pasted
   > `EXPERIMENT=21-32` then scores them into the wrong arm. `SEAT-EXP-6`'s
   > matched-mean check and `SEAT-EXP-7.1`'s pooled sd are computed on mixed
   > arms, CONTROL reads `n=10` and trips `SEAT-EXP-5`, and a sitting with 24
   > good throws in it scores INCONCLUSIVE.
   >
   > With one bag per arm the boundary is physical rather than counted, and the
   > indices restart at 1 in each bag. **Then `--discard` applies only to the
   > burn-in bag** — do not carry `--discard 8` onto a blocks-only bag, it would
   > silently drop the first 8 CONTROL throws. If you do combine both arms into
   > one bag to get the matched-arrival line, derive the ranges from the printed
   > per-attempt table (each row carries its landing time) against the wall-clock
   > time of the flip — never from a count of balls thrown.
   >
   > `--discard` only excludes attempts from the statistics; it still prints them.
   Instrument acceptance first, no bag needed:
   `python tools/probes/ball_arrival_offset.py --self-check` ⇒ `SELF-CHECK: PASS`.
3. **Possession verdicts — REPORT.** Live since `463a031` (contract
   **C-POSSESS-1**):

   ```bash
   python tools/probes/possession_verdict_bag_check.py --bag ~/Desktop/rosbags/<BAG>
   ```
   Expect **0 CAUGHT on the reload path** and that is *correct* — the reload
   tracks are split tracks fed by the wrong marker, so the gate refuses to mint a
   verdict from them (standing rule 3, § SECTION POSS). This row is here to
   confirm the refusal is still clean, **not** to supply the numerator. Any
   *self-toss* in the sitting reading `MISSED` on a watched catch is a finding.

| # | quantity | PASS / REPORT | ABORT |
|---|---|---|---|
| SEAT-EXP-4 | `ball_arrival_offset.py --self-check` before scoring | `SELF-CHECK: PASS`, exit 0, **no `BAD` lines** | any `BAD` ⇒ the instrument is not trustworthy; do not score |
| SEAT-EXP-5 | attempts per arm | **≥ 12** scored in each of CONTROL and EXPERIMENT | fewer ⇒ see the decision rule; the answer is INCONCLUSIVE, not "no difference" |
| SEAT-EXP-6 | **arms are MATCHED** — read the probe's `SEAT-EXP-6 wants gap <= …` line, which computes this against the arms' own noise | gap **≤ max(5 mm, 2·se)** where `se` is the standard error of the *difference of means*, and BOTH means inside the band (−15…−5 mm) | wider ⇒ the arms are confounded by aim, exactly like the last sitting. Report both means and score INCONCLUSIVE |
| SEAT-EXP-7.1 | within-block spread | pooled sd reported; expect **~9–11 mm** (2026-07-27 measured 9.0 mm at the plateau, 11.4 mm over the whole session) | a much larger sd ⇒ the BB is not settled; the burn-in gate was passed too early |
| SEAT-EXP-7.2 | leg peaks on the `0.07` block | within session limits; expect ~`24 / 142 / 3935` against `1000 / 5000 / 30000` | any `MAX_DEVIATION` latch, guard trip or overspeed E-STOP ⇒ **abort the block and revert immediately** |

> **Why `SEAT-EXP-6` is not a flat 5 mm.** A fixed absolute threshold is not a
> statement about whether two blocks match — it is a statement about millimetres,
> and it fires on correct behaviour at this experiment's own numbers. At the
> expected pooled sd of 9–11 mm with `n = 12` per arm, the standard error of the
> difference of means is `s·√(2/12)` = **3.7 mm at s = 9.0** and **4.7 mm at
> s = 11.4**. A 5 mm gate is therefore only `1.4` and `1.1` standard errors out,
> so **two blocks flown at genuinely identical aim exceed it about 17 % and 28 %
> of the time**. That would ABORT a correct experiment roughly once every four to
> six sittings — the same "criterion that fires on correct behaviour" defect that
> got the burn-in gate rewritten from *successive change < 10 mm* to a trend
> slope, and the reason this row is expressed against its own noise instead. The
> `max(5 mm, …)` floor keeps the gate from becoming vacuous if the sd comes in
> unexpectedly small. The probe prints the gap in mm, in standard errors, and the
> resulting `MATCHED` / `NOT MATCHED` verdict, so runbook and instrument express
> the question in the same quantity.

### THE DECISION RULE — pre-registered, 2026-07-28, before the sitting

Written down in advance so the outcome cannot be argued after the fact. `k0` =
bounce-outs in the CONTROL (`0.0`) block, `k1` = bounce-outs in the EXPERIMENT
(`0.07`) block, both out of `n = 12` scored attempts, missed-arrivals excluded
from both numerator and denominator.

**The honest power, computed rather than asserted** — two-sided Fisher exact,
α = 0.05, n = 12 per arm. Reproduce the whole table in a few seconds:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python - <<'PY'
from math import comb
def fisher(a,b,c,d):
    n1,n2,k = a+b, c+d, a+c
    p=lambda i: comb(n1,i)*comb(n2,k-i)/comb(n1+n2,k)
    return sum(p(i) for i in range(max(0,k-n2), min(k,n1)+1)
               if p(i) <= p(a)*(1+1e-12))
n=12
for k0 in range(n+1):
    hit=[f'k1={k1} p={fisher(k0,n-k0,k1,n-k1):.3f}'
         for k1 in range(k0) if fisher(k0,n-k0,k1,n-k1) < 0.05]
    if hit: print(f'k0={k0:2d}: ' + ', '.join(hit))
PY
```

| `k0` (seat `0.0`) | smallest `k1` that is decidable | p |
|---|---|---|
| 5 | 0 | 0.037 |
| 6 | 0 | 0.014 |
| 7 | 0 or 1 | 0.005 / 0.027 |
| 9 | ≤ 3 | ≤ 0.039 |

**So at n = 12 per arm this sitting can decide only a LARGE effect** — the zero
seat must bounce out **at least 5 of 12** while the seat arm bounces out **none**.
`5 vs 1` is *not* decidable (p > 0.05). That is a property of n, not of the seat,
and it is why `SEAT-EXP-2.2`'s marginal band is mandatory: in the marginal regime
the last sitting measured `p0 ≈ 3/4` — 3 bounce-outs in the 4 attempts that
arrived at `x@1000 ≥ −15 mm`, so treat it as an order of magnitude and not an
estimate — which would put `k0 ≈ 9` and make the test comfortably powered; at the plateau `p0 ≈ 0` and nothing is decidable at any n
this sitting can reach. Going to `n = 19` per arm buys almost nothing (it moves
the `k0 = 5 vs k1 = 0` cell from p 0.037 to 0.046) — **offset, not sample size,
is the lever.**

| outcome | verdict | action |
|---|---|---|
| `SEAT-EXP-6` fails (arms not matched, or outside the marginal band) | **INCONCLUSIVE** | shipped default stays `0.0`. Record both block means. Do NOT report "no difference" |
| `(k0, k1)` appears in the table above (seat arm better, p < 0.05) | **SEAT DEMONSTRATED** | flip `trajectory_op.catch_seat_rate_radps` to `0.07` as the shipped default, in its own commit with its own logbook entry. C-CATCH-1 already bounds it (`0.200047 rad/s` at the reload geometry, so `0.07` is unclipped) |
| `k1 ≥ 5` and `k0 = 0` (p < 0.05, seat arm **worse**) | **SEAT HARMFUL** | shipped default stays `0.0`, and this closes the bb-sim deflection premise on hardware. Record it — it is as valuable as the positive result |
| anything else, including `0 vs 0` | **INCONCLUSIVE** | shipped default stays `0.0`. Say plainly that the sitting was underpowered for the effect it saw, and what `k0` it would have needed |

**No result of this experiment changes `planner.py`.** Every branch is a YAML
value plus a rebuild.

#### CHECK SEAT-EXP-8 — revert (mandatory, stage 6)

**Run this before the machine is left, whatever the verdict** — unless the
verdict was SEAT DEMONSTRATED *and* the operator has decided to ship `0.07`, in
which case the flip lands as a reviewed commit, not as a leftover working-tree
edit.

```bash
# 1. edit config/hardware_config.yaml -> catch_seat_rate_radps: 0.0
source ~/Desktop/PDJ_venv/venv/bin/activate
python config/generate_config.py
git diff --stat                          # expect ONLY the seat-rate line + generated
cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && cd ..
# 2. RELAUNCH.
# 3. re-run SEAT-EXP-1 — all three rows must PASS again.
python -m pytest tests/motion/test_trajectory_planner_catch.py \
                tests/motion/test_catch_reach_replay_probe.py \
                tests/ros/test_trajectory_node.py -q
```

| # | quantity | PASS | ABORT |
|---|---|---|---|
| SEAT-EXP-8.1 | `SEAT-EXP-1.1/1.2/1.3` re-run | all three PASS | any failure ⇒ the revert is incomplete; **do not leave the machine** |
| SEAT-EXP-8.2 | the three test files above | green | the 6 expected-red tests are still red ⇒ the regenerate was skipped |
| SEAT-EXP-8.3 | `git status` | no stray edits outside `config/hardware_config.yaml` and the generated consumers | anything else ⇒ somebody edited source at the bench, which the config key exists to prevent |

### Not in this section

- **Changing what a catch commands beyond the seat rate.** The reach shape, the
  0.15 s decay, `hold_after`, the settle hold and C-CATCH-1's bound are all
  untouched. The only variable is the fallback rate.
- **Fixing the BB warm-up drift.** The burn-in *works around* it; it does not
  explain or fix it. If the drift is worth removing, that is Ball-Butler work.
- **The tracker's split-track corruption**, which is why the reload `outcome`
  cannot supply the numerator. Separate open investigation.
- **The hand's end-stop margin above ~0.8 m throws.** Unrelated to the reload
  path this section exercises, but it is the other thing the last sitting said
  should stop a future one — see § RESIDUAL RISK before adding tosses to this
  sitting.
