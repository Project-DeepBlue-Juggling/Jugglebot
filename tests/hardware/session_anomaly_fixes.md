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

1. **POWER-CYCLE THE CAN-BRIDGE TEENSY before the sitting**, and log `uptime_ms`
   alongside **every** timing measurement (achieved flight, catch error, `shift`,
   any inter-arrival gap). Tracking lag grows with that board's uptime — 10 ms at a
   fresh boot to ~240 ms at 30 h — so a timing number without an `uptime_ms` beside
   it is not interpretable.
2. **CHECK the correction after every launch; `level` only if it is missing.**
   *(Corrected 2026-07-27. This rule previously read "run a manual `level` after
   every launch and every relaunch". That was wrong — it would cost a needless
   levelling routine on every build gate, and it mis-states two of this file's own
   checks. The correction is recorded rather than silently swapped because the
   wrong version is the kind a future session would inherit.)*

   The correction inside `trajectory_node` **is** per-process, but it is normally
   **restored automatically on ROS2 boot**. `RobotState` carries both
   `levelling_complete` **and** `pose_offset_rad` from the **Platform** Teensy
   (`teensy_bridge_node.py:1430`); the orchestrator stores both
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
   > `true`, the check's precondition was never established: the reliable way to
   > force the un-levelled state is a **Platform Teensy** power-cycle (its cache is
   > "since last bootup"), not the can-bridge power-cycle standing rule 1 mandates.
   > Record which case you got — that observation settles the race question for
   > every future sitting, and is worth more than the checks themselves.
3. **The tracker still reports `MISSED` on real catches.** Judge every catch **by
   eye** as well as by `outcome`, everywhere in this file. Record one truthful
   outcome line per attempt.
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

| | what changed | what you must do | **how you find out you skipped it** |
|---|---|---|---|
| **A** | Python under `ros_ws/src/jugglebot/**` — §§ FK, HAND-1, HAND-2, HAND-3, LVL, CCATCH, ZSEAT (commits `aea7b49`, `e58ed89`, and the hand phases) | `colcon build --packages-select jugglebot` + `source install/setup.bash` + **relaunch** `jugglebot_launch.py` | **Loudly, if you run the pre-flights.** Each affected section has a grep against the *installed* copy that prints `PF<n>_STALE` on the run sheet (PF-1…PF-4, stage 3) and `INSTALLED_STALE` in the per-section pre-flights — two token spellings for one check, so match on the `STALE` suffix, not the whole word. Skip the pre-flight and the section silently re-measures the pre-fix baseline and you score a working fix as broken |
| **B** | `jugglebot_interfaces` — `TrajectoryStatus.msg` gained `gravity_correction_loaded` (§ Section LVLGATE, commit `e36d60d`) and `RobotState.msg` gained `platform_fw_version` / `platform_fw_version_read` (§ Section FW) | `colcon build --packages-select jugglebot_interfaces jugglebot` + `source install/setup.bash` + **relaunch**. **Building only `jugglebot` is NOT enough** | **Loudly and catastrophically, now from two nodes.** `_publish_status` assigns a field the generated message's `__slots__` lack, raising inside the 0.2 s timer; rclpy re-raises timer exceptions out of `spin()` and `main` catches only `KeyboardInterrupt`, so **`trajectory_node` EXITS ~200 ms after launch**. You see: no `trajectory_node` in `ros2 node list`, no 40 Hz hold stream, `ros2 topic echo /trajectory/status` hangs, and **`activate` FAILS at the A2 arm ("no mpccmd frame")** — you never reach TRAJECTORY, so you never send a toss at all. LG-0 catches it in 3 s. **`teensy_bridge_node` behaves DIFFERENTLY — do not expect it to exit.** Its 100 Hz `_publish_robot_state` assigns the two new `RobotState` fields but *catches its own exceptions*, so a half-rebuild there gives you **one throttled `Robot state publish error:` per 5 s and a silently-dead `/robot_state`** — the node stays in `ros2 node list` looking healthy while the orchestrator stalls in BOOT and blames power/CAN. Since 2026-07-27 it also logs, once at construction, `INTERFACES_STALE: …` naming the missing fields and the exact rebuild command — **grep that first** (`grep INTERFACES_STALE "$LOG"`). Note this matters most when `jugglebot_interfaces` is only *partly* stale: if it already carries `gravity_correction_loaded` from an earlier sitting, `trajectory_node` does NOT exit and the loud row-B signature above never appears |
| **C** | `ros_ws/src/jugglebot/Teensy_code/Trajectory.h` + the regenerated `Teensy_code/hardware_config.h` (§ CHECK HAND-4, commit `5369fc2`), and `Teensy_code.ino`'s `FW_VERSION` identity block (§ Section FW) | **FLASH `Teensy_code/Teensy_code.ino` to the PLATFORM Teensy.** Not the can-bridge (`Teensy_code_canbridge/`), not the CatchingCone. `colcon build` does not touch it and the Jetson never executes it | **Loudly, since 2026-07-27 — read the box below.** `link_status/platform_fw_version` reads `0 (PRE-VERSIONING)` on an un-flashed board and `1` on a flashed one, and the launch log carries a `PLATFORM_FW_CHECK: FAIL` ERROR. Run-sheet row **FW-1** |

> ### ⚠ SUPERSEDED (2026-07-27): the un-flashed Platform Teensy is now DETECTABLE
>
> This box used to read *"THE HEADLINE: an un-flashed Platform Teensy is
> UNDETECTABLE from the Jetson"* and hand you a **four-link circumstantial chain**
> (right source → the header compiles → you uploaded → the board rebooted) because
> the board carried no `FW_VERSION` and nothing on the wire could tell you. **That
> chain is superseded. Do not run it — run FW-1 instead.** It was inference about
> your own actions; FW-1 is an observation of the board.
>
> The board now declares `FW_VERSION` (`Teensy_code.ino`, currently **1**) and
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
> | `PLATFORM_FW_CHECK: OK — … v1` / `1` | flashed, current | continue |
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

Score the reload and toss halves of CAP-WORK with **separate probe invocations**:
`--thrower ball_butler` for the reloads, `--thrower jugglebot` for the tosses.

## THE RUN SHEET

> ### ✅ EXECUTED 2026-07-27 — outcome, and what to fix before the next run
>
> Stages 1–7 were run (can-bridge Teensy rebooted; Platform Teensy **flashed** and
> `FW-1` confirmed `v1` on all six launches), then the `session_phase8_toss_hardware.md`
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
>   Teensy truncates the stored offset to **int16 milliradians** (`Teensy_code.ino:430`,
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
> pinned (three sources disagree: 11.124 / 11.224 / 11.4 rev) and `FLIGHT_TIME_MAX_S`
> is reconciled with it.

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
                tests/firmware/test_platform_fw_version_xref.py -q   # INST-4
cd ros_ws/src/jugglebot/Teensy_code && pio run && cd ~/Desktop/Jugglebot   # INST-5
```

| # | PASS | ABORT | routes to | detail |
|---|---|---|---|---|
| INST-1 | `SELF-CHECK: PASS`, **10/10 `OK`**, exit 0. Case 7 must show `planner._CATCH_TILT_THROUGH_RATE_RADPS=0.0` **and** `recorded-session rate (capture record, NOT a live mirror)=0.07` | any `BAD` | `catch-reach-degenerate-overshoot` P0/P2/P3 | § CCATCH-1, § ZSEAT-1 |
| INST-2 | `SELF-CHECK: PASS`, exit 0 (scores a synthetic post-fix session PASS, a pre-fix session FAIL, an ACTIVATE-contaminated session FAIL-with-note) | anything else | `levelling-frame-contract` P1–P2 | § LVL-0 |
| INST-3 | exit 0 and **TWO** `GATE PASS` lines: `25/25 rows within tolerance` **and** `fixed-shape branch`. **Judge on the exit code and both lines, not the row count** — the count grows whenever a reference row is added and has already produced one stale runbook | `GATE FAIL`, a missing second line, or non-zero exit. `GATE UNAVAILABLE` is different — the fixture is missing; restore or regenerate it | `hand-command-continuity` P0 | § The analysis command |
| INST-4 | `passed`, with **ZERO skips** | any failure, or `passed, N skipped` — a SKIP means `g++` was absent and the only two things that read the C++ read nothing. **Do not flash on a skip** | `hand-command-continuity` P4 / P6 | § H4.0b, § Section FW |
| INST-5 | `[SUCCESS]` — the WHOLE Platform sketch compiles and links for the Teensy 4.0 | any error ⇒ **do not open the sketch, do not flash**; the source you are about to flash does not build. Needs network on a cold PlatformIO cache (~1 min); warm it is ~10 s | `hand-command-continuity` P6 | § Section FW |

INST-5 is a **compile gate only** — `Teensy_code/platformio.ini` has no
`upload_command` on purpose and physically cannot flash the board (see its
header). It does not replace the Arduino IDE flash in stage 2; it proves the
sketch builds before you get there. Before 2026-07-27 nothing in the repository
compiled this sketch at all. It drops a `.pio/` build tree inside `Teensy_code/`
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
  ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code/Trajectory.h
```

| # | PASS | ABORT | routes to |
|---|---|---|---|
| PF-1 | `PF1_OK` | `PF1_STALE` | `fk-convergence-tolerance` P1 (§ Section FK pre-flight) |
| PF-2 | `PF2_OK` | `PF2_STALE` | `hand-command-continuity` P1 (§ HAND-0) |
| PF-3 | `JB_OP_HAND_CATCH_PRIME_REV = 9.9594` **and** `HAND_STROKE_TOP_REV = 9.95940313273228` | `9.858`, or `HAND_STROKE_TOP_REV` absent | `hand-command-continuity` P3 (§ HAND-3a) |
| PF-4 | `PF4_OK` then `0` for `_apply_gravity_correction` | `PF4_STALE`, or a non-zero count (the deleted second copy is back) | `levelling-frame-contract` P1–P2 (§ LVL-0) |
| PF-5 | all three `gravity_correction_loaded` / `NOT_LEVELLED` counts **non-zero** (at the Phase-3 commit: `1`, `3`, `2` — treat the exact numbers as informational, **zero** is the failure) | any count `0` | `levelling-frame-contract` P3 (§ LG-0) |
| PF-6 | `1` hit for `start_vel = current_hand_velocity` | `0` — you are on a pre-Phase-4 tree; **do not flash it**, `git pull` first | `hand-command-continuity` P4 (§ H4.0a) |

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
| FW-1 | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v1`, and `link_status/platform_fw_version` = `1` | `FAIL … PRE-VERSIONING` or `0 (PRE-VERSIONING)` ⇒ **the Platform Teensy was NOT flashed** — go back to stage 2, flash, relaunch, re-run FW-1. `FAIL … v<other>` ⇒ flashed from a different tree; `git pull` + re-flash. `UNKNOWN` / `unknown` ⇒ no read landed; this is usually the **known benign boot-read transient**, so **relaunch once and re-read** before investigating CAN3 (co-signature: `cold-start boot read failed after N attempts`, `cold_start_authoritative` = `0`). **No line at all** ⇒ you skipped `colcon build`; rebuild both packages and relaunch — never score an absent `FAIL` as a pass | `hand-command-continuity` P6 (§ Section FW) |
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

**Leave `catch/vel_scale` at its 0.8 default all sitting.** It multiplies the armed
event velocity and the arm window's right edge is `0.404 / v_armed`, so a *low*
scale lengthens the required lead and closes the window on a perfectly healthy
tracker. Swept against the production velocities: at a 0.55–0.56 s flight, **0.45
closes it (−15 ms)** and 0.50 barely opens it (+18 ms), while 0.8 gives +116 ms; at
0.80 s and above the window stays open across the whole shipped `[0.3, 1.5]` range.
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
| HAND-1 | the catch arm no longer lands inside the throw stroke | rows 1–5 of § PASS / ABORT per throw on **every** toss, plus H1.2–H1.7 | any row ABORTs — **EXCEPT row 4 (`dip_below_x3`) on a toss where row 7 (`first_neg_cmd`) is annotated**: a braking prelude fired, the two rows score the same event in opposite directions, and row 4 becomes **REPORT** (score it against the brake's own turning point — see HAND-4). `Not enough time for smooth-move` on the Teensy serial is a **hard section abort** | `hand-command-continuity` P1, § HAND-1 |
| HAND-2 | a repack under a failed ack does not clobber a live stroke | `arms` is 1 or 2 (never ≥ 3); `seeds = 0` on every `arms == 2` toss | `seeds >= 1` on an `arms == 2` toss. If **no** toss reads `arms == 2`, say so — the criterion was never exercised | `hand-command-continuity` P2, § HAND-2 |
| HAND-3 | the hand parks at the derived stroke top and nothing misjudges it | H3.1–H3.7 as tabulated; prime `pos_meas` inside **[9.4594, 10.4594]**, `Hand primed to 9.959 rev`, peak prime `vel_meas` **≤ 30 rev/s** against a commanded quintic peak of **24.63 rev/s** | outside the near-band; `9.858 rev` in the log (stale install); `>= 40.0 rev/s` | `hand-command-continuity` P3, § HAND-3 |
| HAND-4 | the flash did not break the clean path | identical to HAND-1 (**that is the designed PASS**), `peak <= 10.060` rev (no pre-flash control on this run sheet — see above), no commanded `pos < 0.0` rev, no commanded move longer than **0.8005 s**. `first_neg_cmd` annotated `<-- NOT the catch descent (a brake?)` is **REPORT, not abort** — and on such a toss score `dip_below_x3` against the brake's own turning point, not against `x3` | H4.4 / H4.5 / H4.7 / H4.8 / H4.9 / H4.10 as tabulated. **`peak > 10.60` rev is a HARD ABORT + E-STOP** | `hand-command-continuity` P4, § HAND-4 |
| LVL-3 | **the levelling headline** — the frames agree across a goal | `VERDICT: PASS`: the **park** plateau within **±0.05°** of `(−tilt_x, −tilt_y)` on both axes. **`--t0` is not optional, and its unit is SECONDS FROM BAG START** (a float, not wall-clock, not ROS epoch). Run once *without* `--t0`, read the plateau table's `t_start` column, then re-run with `--t0` a few seconds past the LVL-2 `go_home` plateau | `park_rx ≈ 0.0000°` with no `NOTE:` (pre-fix frame); or ≈ **−1.5576°** (twice the correction — applied twice) | `levelling-frame-contract` P1–P2, § LVL-3 |
| LVL-4 | mocap cross-check (does not share the FK path) | **REPORT-ONLY since 2026-07-27 — it is no longer a gate, and the old `±0.10°` PASS is the PRE-fix reading.** Run § LVL-4's inline reader and record the parked Platform-vs-`Base` tilt; expect it to have moved BY the correction, to **≈ 0.78°** (pre-fix baseline **0.087°**) | nothing here aborts on its own. `≈ 1.56°` (twice the correction) is worth stopping for — **confirm on LVL-3 first**, which is the instrumented, gated version of the same question | `levelling-frame-contract` P1–P2, § LVL-4 |
| CCATCH-2 | **the catch-reach headline** — a level catch commands NO swing | commanded `rx` across the pre-tilt reach **monotone** toward the target, peak above park ≤ `1.05 ×` the requested displacement; toss settle `rx`/`ry` = the target to **±0.05°**; residual vs gravity at contact **≤ 0.05°**; plan segments **2**; `peak_leg_acc/jerk` **≈ 1.2 / ≈ 3** (was `142.4 / 3950`) | any excursion **away** from the target > `0.05°`; settle at `−1.0784 / −0.0958°` (the old aim is live); 3 segments; still `≈142 / ≈3950` | `catch-reach-degenerate-overshoot` P2, § CCATCH-2 |
| CCATCH-2t | tracker catch error on a **self-toss** | **< 10 mm** | ≥ 16 mm — the improvement did not land. **Judge by eye too** (standing rule 3) | `catch-reach-degenerate-overshoot` P2, § CCATCH-2 |
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
Score H1.1–H1.7 only. **Run it at the default `catch/vel_scale` (0.8)**: a reduced
scale closes the window at this flight length by itself and you would be measuring
the knob, not the gate. A MISSED catch here is not by itself a Phase-1 failure.

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
   `Teensy_code.ino:472-475` checks before `packedMsgs.clear()`), but the gap itself
   remains, and a pre-release SAFE_ABORT still depends on a kind-3 retract being
   able to clobber an armed stroke. H2.4 / H3.6 / H4.8 watch it; none of them
   provokes it.
6. **A velocity-continuous prelude is LONGER than the rest-to-rest one** — 0.24 s at
   the 6.0 rev/s dead-band edge, 0.32 s at 8 rev/s, against the 76 ms
   `PRELUDE_ALLOWANCE_S` budget. So at `FLIGHT_TIME_MIN_S` the firmware's own `:533`
   fit check could refuse the arm — a lost catch with the live stroke intact. It is
   reachable only if the hand is drifting 6–9 rev/s when a kind-1 arm lands, against
   a measured settle tail of ≤ 0.25 rev/s, so it has never been observed. Row
   **H4.7** watches for it; contract limit **F.1** records it.
7. **An operator decision this run deliberately did NOT take, with its numbers.**
   Phase 4 found the plan's premise physically unshippable: arresting `v0` costs
   `0.0077832·v0²` rev of travel, so velocity continuity is affordable only to
   **~9.1 rev/s** at the stroke top and **~20.9 rev/s** mid-stroke (**~20.3 rev/s**
   once the 0.8005 s duration cap binds), while the hand passes release at
   **~120 rev/s** — needing **111 rev** of arrest travel against **11.1 rev** of
   stroke. Both plan options were rejected with numbers: refusing the command breaks
   the kind-3 clobber (the only un-arm mechanism), and braking hard enough would
   need **28 000 rev/s²** on a mid-descent retract, **280×** the declared limit. It
   shipped a third: fall back to the rest-to-rest profile — *today's exact
   behaviour*, adding no commanded magnitude the firmware could not already produce.
   **The open question**: `MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2 = 100 rev/s²` is a
   **comfort** limit, and the shipped throw profile itself commands **1908 rev/s²**
   at a 0.80 s flight and **6055 rev/s²** at the band top — so the hand has 19–60×
   more authority than the smooth move is allowed to use. Whether `makeSmoothMove`
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
> `ros_ws/src/jugglebot/Teensy_code/Trajectory.h`, which is compiled into the
> **Platform Teensy** sketch. `colcon build` does not touch it and the Jetson never
> executes it, so **HAND-4 requires flashing `Teensy_code/Teensy_code.ino` to the
> Platform Teensy** (not the can-bridge, not the CatchingCone). Since 2026-07-27
> the board carries a `FW_VERSION` and reports it, so a skipped flash IS
> detectable — run-sheet row **FW-1**, and § CHECK HAND-4 row **H4.0d**. It is
> reported, not enforced: nothing refuses a command on a skew, so FW-1 is a check
> you must actually run. Every other section in this runbook is colcon + relaunch;
> `levelling-frame-contract` Phase 3 additionally needs `jugglebot_interfaces`
> rebuilt (see § Section LVLGATE).

### Recording — ONE list, for every capture

Individual sections below each say "append these topics". **Do not run two
different lists.** Extra topics are harmless to every analysis command in this
file, and a missing topic is unrecoverable after the fact — several of these
measurements *cannot* be reconstructed from a bag that lacked them. Use this
consolidated command for **every** capture (CAP-GATE, CAP-RELAUNCH, CAP-WORK,
CAP-SHORT):

```bash
mkdir -p ~/Desktop/rosbags && cd ~/Desktop/rosbags
ros2 bag record -o "$(date +%Y-%m-%d_%H-%M-%S)" \
  /robot_state /leg_setpoint_echo /platform_target /rigid_body_poses \
  /link_status /rosout /trajectory/status \
  /trajectory/diagnostics /trajectory/target_feedback \
  /catch/dynamic_target /gravity_offset /throw_announcements \
  /hand_telemetry
```

Note the bag directory name — the analysis commands take it as `--bag`.

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

**Plan**: `plans/active/fk-convergence-tolerance.md` § Phase 1
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

**Plan**: `plans/active/hand-command-continuity.md`

Phases 1, 2 and 4 append their own `CHECK HAND-n` bodies under this header as
they land. This first part is the **shared instrument and its pre-fix
baseline** — the numbers a post-fix capture is scored against. It exists here
because the baseline was measured offline from three 2026-07-25 sessions and
lives nowhere else; without it the Phase-5 criterion (`dip_below_x3 <= 0.10` rev,
row 4 below) has nothing to compare to.

**Nothing in this part actuates the robot.** It is read-only analysis run
*after* a capture that a later `CHECK HAND-n` produces.

### Capture requirement for every HAND check

The dip lives in `hand_telemetry`, which the § Recording bag list above does
**not** include. Run the toss-trace recorder alongside the bag, in its own
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

**The trace recorder is not optional for a HAND check.** The § Recording bag
command above records `/robot_state /leg_setpoint_echo /platform_target
/rigid_body_poses /link_status /rosout` — **neither** `/hand_telemetry` **nor**
`/throw_announcements`, which are the two topics this probe reads for the stroke
timeline. Run the probe's `--bag` path only against a bag recorded with both
topics added:

```bash
# optional: a bag that ALSO feeds the HAND probe (append to the § Recording list)
  /hand_telemetry /throw_announcements
```

Such a bag **does** carry `/rosout` (the § Recording command records it), and the
probe reads the arm-dispatch count from it — it detects the channel rather than
assuming the format. `arms` reads `?` only when the source genuinely has no
`/rosout`, which is true of the three 2026-07-25 evidence bags (recorded before
that list) but not of a bag recorded with the command above. `?` never means zero;
the launch log `~/.ros/log/<stamp>/launch.log` is the fallback source.

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
  branch (`25/25 rows within tolerance` as of 2026-07-26) and one for
  `fixed-shape branch` (**five** cases: `clean`, `overshoot`, `short-flight`,
  `braking-prelude`, `deep-brake` — the fifth was added by this run's Phase 4 as
  the separator at the excursion clamp's reach, so a probe printing only four is
  a pre-`5369fc2` tree). Judge on the exit code and on both lines being `GATE
  PASS`, **not** on the row count: the count grows whenever a row is added to
  the reference table, and treating it as the criterion has already produced one
  stale runbook.
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
| 1 | `trunc` | `-` (the command followed the decel ramp to `x3`) | any instant printed ⇒ the queue was still cleared mid-stroke |
| 2 | `seeds` | `0` (printed as `-`) | `>= 1` from-rest quintic seed inside the stroke |
| 3 | `peak` | `<= 10.060` rev (`x3` 9.9594 + 0.10) | `> 10.060` rev. **Corrected 2026-07-27: the hard abort is `> 10.60` rev, not `> 10.5`** — 10.60 rev is the excursion clamp's own ceiling (`11.1 − 0.5`), and `10.060 < peak <= 10.60` is a **section** abort with a specific suspect (H4.4), *not* an E-STOP. `> 10.60` rev is the HARD ABORT + E-STOP (H4.5). One number, one response |
| 4 | `dip_below_x3` | `<= 0.100` rev (`<= 3.2` mm) — the row prints `OK` | `> 0.100` rev — the row prints `OVER`. Pre-fix range was **0.339–1.748 rev = 10.7–55.3 mm**. **Qualified by row 7 after Phase 4** — see below |
| 5 | `pullback` | `>= -5.0` rev/s, **given row 3 passed** | `< -5.0` rev/s. Pre-fix range was **−17.9 to −42.4 rev/s** |
| 6 | `catch_desc` | present, within ~20 ms of `event − t_acc_catch` | absent ⇒ the catch never fired; check the Teensy serial for `Not enough time for smooth-move` |
| 7 | `first_neg_cmd` | equal to `catch_desc` (no annotation printed) | annotated `<-- NOT the catch descent (a brake?)`: **REPORT, do not abort.** Expected after Phase 4 lands (step 3 charters a braking prelude); before Phase 4 it means an unexplained downward command. Phase 4's measured reality narrows this: a braking prelude only appears once the hand's live `\|vel\|` exceeds the 6.0 rev/s dead-band, and the settle tail after a completed stroke reads `<= 0.25` rev/s, so on a clean capture this row should still read `catch_desc`. See § CHECK HAND-4 |

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
goes under (the four synthetic post-fix shapes read 0.000–0.001 rev). Same reason
`pullback` is bounded rather than required non-negative: a healthy settle from the
coasting peak is genuinely negative — −0.31 rev/s at 0.02 rev of overshoot, −1.58
at the 10.060 rev ceiling of row 3, −10.03 at 10.60 rev.

**Row 3 governs; 10.60 rev is a CEILING, not an expectation.** The probe's
`overshoot` synthetic and the `pullback` figures above run out to 10.60 rev
because that is where Phase 4's excursion clamp caps a velocity-continuous
prelude (`11.1 − smooth_move_excursion_margin_rev 0.5`). A clean post-fix capture
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

| session | ball | arms | `trunc` (rev) | `peak` (rev / mm) | headroom to 11.1 | `dip` (mm / % stroke) | **`dip_below_x3` (rev / mm)** | **`pullback` (rev/s)** | `shift` (ms, **bag clock**) |
|---|---|---|---|---|---|---|---|---|---|
| 15-04-35 | 34 | 2 | 7.1245 | 10.2611 / 324.5 | 0.839 rev | 20.3 / 6.4 | **0.339 / 10.7** | −17.9 | +12.8 |
| 15-17-48 | 10 | 1 | 6.7562 | 10.2513 / 324.2 | 0.849 rev | **64.5 / 20.5** | **1.748 / 55.3** | −42.4 | +19.0 |
| 15-17-48 | 11 | 2 | 6.1965 | 10.2684 / 324.8 | 0.832 rev | 52.4 / 16.6 | **1.347 / 42.6** | −36.6 | +20.7 |
| 15-17-48 | 13 | 1 | 7.1897 | 10.2813 / 325.2 | 0.819 rev | 56.6 / 18.0 | **1.468 / 46.4** | −38.2 | +15.4 |
| 15-17-48 | 17 | 2 | 6.8525 | **10.3248 / 326.6** | **0.775 rev** | 23.0 / 7.3 | **0.361 / 11.4** | −20.0 | +20.2 |
| 15-22-50 | 2 | 1 | 7.7825 | 10.1653 / 321.5 | 0.935 rev | 40.2 / 12.8 | **1.065 / 33.7** | −29.7 | +17.2 |
| 15-22-50 | 3 | 1 | 7.7004 | 10.1743 / 321.8 | 0.926 rev | 43.0 / 13.7 | **1.146 / 36.2** | −31.3 | +21.9 |

Worst pre-fix case: **55.3 mm below the stroke end** (ball 10) and **0.775 rev =
24.5 mm** of headroom to the 11.1 rev end-stop (ball 17), both at a mid-band
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
| H1.1 | `trunc`, `seeds`, `dip_below_x3`, `peak`, `pullback` per toss | rows 1-5 of § PASS / ABORT per throw, on **every** toss | any row ABORTs |
| H1.2 | `window latched` count | **== number of jugglebot tosses** | `0` ⇒ the announcement never reached the gate (wrong `thrower_name`, or the announcement arrived before `catch/armed`); the dip may be absent for an unrelated reason and the PASS is luck |
| H1.3 | `arm withheld` count | **>= 1 per toss** | `0` while H1.2 passed ⇒ the arm was already late on its own; record it, and treat any H1.1 PASS as unvalidated for the gate |
| H1.4 | `stroke-busy window CLOSED` warnings | **0** | `>= 1`: the fit check refused to defer. Not a *new* hazard — the branch reproduces the pre-fix arithmetic exactly — but it does **not** mean the catch fired: the forced dispatch may itself be refused by the Teensy (see H1.6), because its fit check budgets the at-rest prelude while the hand is mid-stroke. Record `event_delay`, `event_vel` and `vel_scale` from the warning, then **check `catch/vel_scale` FIRST** (see the note below) before routing to `hand-command-continuity` Phase 1 step 3 |
| H1.5 | the withheld line's reported slack | **> 0.050 s** on every line | `<= 0.050 s` ⇒ the window is tighter in practice than the 115 ms modelled floor; capture and route to Phase 1 step 3 before running more tosses. This row is also the guard on the ~23 ms bridge→Teensy transit that `required_arm_lead_s` deliberately does not model |
| H1.6 | `Not enough time for smooth-move` on the Teensy serial | absent | present ⇒ the arm was refused wholesale and that toss's catch never fired. **Hard ABORT of the section.** Note the refusal leaves the throw stroke INTACT (`:533` returns before `packedMsgs.clear()`), so this can co-occur with a clean H1.1 dip row — a clean dip is NOT evidence the catch happened |
| H1.7 | every toss that logged a `hand catch arm withheld` line also shows a later `Arming hand catch` (equivalently, probe `arms >= 1` on that toss) | **every withheld toss redeemed** | any withheld toss with no later dispatch ⇒ the deferral was never redeemed: the balls tick that should have dispatched it never arrived (track dropout, or a landing revision pushing `event_delay` under the 0.3 s floor — both bypass the gate, so no CLOSED warning appears either). Not a safety abort, but it is the one way withholding can turn into dropping — record it and route to `hand-command-continuity` Phase 1, which would then need a one-shot timer rather than a tick-driven retry |

**Before routing any H1.4 CLOSED warning to a tracker fault, read
`catch/vel_scale`.** The knob multiplies the armed event velocity, and the
window's right edge is `0.404 / v_armed`, so a LOW scale lengthens the required
lead and closes the window on its own with a perfectly healthy tracker. Swept
against the production velocities: at the 0.55-0.56 s flight, scale **0.45
closes it (−15 ms)**, 0.50 barely opens it (+18 ms), the 0.8 default gives
+116 ms; at 0.80 s and above the window stays open across the whole shipped
`[0.3, 1.5]` range. That corner is exactly HAND-1b below, so a CLOSED warning
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
**115 ms** at the band floor `FLIGHT_TIME_MIN_S = 0.55 s`. Two tosses at

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.38}" --feedback
```

(0.38 m = T 0.557 s, just inside the band; `> 1.48 m` and `< 0.371 m` are
`REJECTED_FLIGHT_TIME`). Same verdict rows. **This is the corner where H1.4 is
most likely to fire**, which is the point of running it — and § Height reference
already flags `T < 0.7 s` as stroke-marginal for reasons unrelated to this plan,
so a MISSED catch here is not by itself a Phase-1 failure. Score H1.1-H1.7 only.

**Run this at the default `catch/vel_scale` (0.8).** A reduced scale closes the
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
| H2.2 | probe `seeds` on any toss with `arms == 2` | `0` (printed `-`) — **the Phase-2 criterion**: both dispatches landed clear of the stroke | `>= 1` ⇒ a repack still clobbered a live stroke. Pre-fix, every `arms=2` toss showed exactly 2 seeds, `0.0000 rev` from the live `pos_meas` |
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
matters because the generated `Teensy_code/hardware_config.h` *did* change:
`JBOp::HAND_CATCH_PRIME_REV` is referenced by **no** `.ino`/`.h`/`.cpp` in any of
the three sketches — it is a dead `constexpr` that codegen delivers for
completeness. Verified by grep across `Teensy_code/`, `Teensy_code_canbridge/` and
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
`plans/active/hand-command-continuity.md` **Phase 3** on failure.

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
| H3.4 | `grep -i 'smooth_move_hand' launch.log \| grep -iE 'reject\|out of range'` | `0` hits | `>= 1` ⇒ the target left the bridge's `[0, 11.1]` validation range. Structurally impossible at 9.9594 (headroom **1.1406 rev = 36.1 mm**); a hit means the YAML override was mis-typed |
| H3.5 | peak `pos_meas` during/just after a prime ascent | `<= 10.25` rev (1.6x the measured +0.186 rev overshoot at the old prime) | `>= 10.60` rev — still 0.5 rev short of the 11.1 overextension guard, but the prime is now 0.1014 rev closer to it than it was, so this is the row that watches that. **DEBRIEF (not abort) in `10.25 < peak < 10.60`**: overshoot beyond 1.6x the measured baseline with the end stop 0.5-0.85 rev away — record the peak and route to Phase 3 before further tosses |
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
> not.** Phase 4's whole deliverable is in `Teensy_code/Trajectory.h`, which is
> compiled into the **Platform Teensy** sketch — the board that drives the hand
> via the can-bridge conduit. A relaunch, a rebuild, or both, change **nothing**
> about this check: `Trajectory.h` is not Python and the Jetson never executes it.
>
> Flash **`ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino`** to the **Platform
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

**Plan**: `plans/active/hand-command-continuity.md` § Phase 4
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
cd ~/Desktop/Jugglebot && git log --oneline -1 -- ros_ws/src/jugglebot/Teensy_code/Trajectory.h
grep -c 'current_hand_velocity;' ros_ws/src/jugglebot/Teensy_code/Trajectory.h   # expect 1
grep -n 'start_vel = current_hand_velocity' ros_ws/src/jugglebot/Teensy_code/Trajectory.h
grep -n 'SMOOTH_MOVE_V0_DEADBAND_RPS' ros_ws/src/jugglebot/Teensy_code/hardware_config.h

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
| **H4.0d** | `grep PLATFORM_FW_CHECK "$LOG"` — **the direct check; this is the one that decides** | `PLATFORM_FW_CHECK: OK — Platform Teensy reports v1`. Equivalently `ros2 topic echo /link_status --once` shows `platform_fw_version: 1` | `FAIL … PRE-VERSIONING` ⇒ **the board was not flashed; every row below is meaningless.** Flash and relaunch. `FAIL … v<other>` ⇒ flashed from a different tree. `UNKNOWN` ⇒ no read landed — **not** a stale flash, and usually the known benign boot-read transient: **relaunch once and re-read**, investigate CAN3 only if it repeats. **No line at all** ⇒ the `colcon build` was skipped; rebuild both packages and relaunch. This ROW SUPERSEDES the old four-link inference chain: H4.0a–c are about the tree you flash FROM, this is about the board you flashed TO. Contract: `ros_ws/docs/platform_fw_version.md`; phase: `hand-command-continuity` P6 |

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
| H4.5 | `peak > 10.60` rev | does not occur | **HARD ABORT, E-STOP.** The clamp is `11.1 − 0.5 = 10.60` rev *commanded*; exceeding it means the clamp did not run (flash suspect — re-check H4.0), or the position loop overshot the commanded profile by more than the 0.5 rev margin (2.7× the +0.186 rev tracking overshoot measured at the old prime), or the documented endpoint relaxation served a prelude from a live position *already* above 10.6 rev — which is the pre-fix measured state (10.165–10.325 rev), is legal by design, and never adds a bulge above that live reading. Check `first_neg_cmd` and the live `pos_meas` at the command instant to tell them apart. Either way the next 0.5 rev is the overextension guard and the 0.76 mm after that is the hard stop |
| H4.6 | `seeds` on any toss, cross-read with H4.3 | `0` (printed `-`) | `>= 1`: same ABORT as HAND-1 row 2, but Phase 4 adds a second suspect. A from-rest seed now means **either** the arm gate failed (Phase 1) **or** `makeSmoothMove` took its documented cannot-fit fallback — because the excursion would have left the stroke, *or* because the arrest would have taken longer than 0.8005 s. Distinguish by the seed's position: a seed *inside* the decel ramp (below `x3`) with `trunc` printed is the Phase-1 failure; a seed at or above `x3` with `trunc = -` is the fallback. Record which, and the `vel_meas` at the seed — above ~9 rev/s at the stroke top it is the excursion clamp, above ~20 rev/s anywhere it is the duration cap |
| H4.7 | `Not enough time for smooth-move` on the Teensy serial | absent | present ⇒ same hard abort as H1.6. Note Phase 4 can *lengthen* a prelude (a velocity-continuous move takes longer than a rest-to-rest one over the same Δ — up to 0.24 s at the dead-band edge, 0.32 s at 8 rev/s), so if this appears **only** after the flash and H4.3 shows a brake on the same toss, the arm-fit budget needs the continuous prelude added. Route to `hand_stroke.required_arm_lead_s` |
| H4.8 | a SAFE_ABORT retract, **if one occurs naturally** | the retract still runs and the hand reaches `\|pos\| <= 0.5` rev | it does not. **Hard ABORT** — a kind-3 retract clobbering an armed kind-0 is the only un-arm mechanism the Teensy offers, and Phase 4 edits the exact condition (`makeSmoothMove` returning empty) that `Teensy_code.ino:472-475` checks *before* `packedMsgs.clear()`. The change **narrows** that branch (empty now also requires the hand to be at rest), so this should be strictly safer than before the flash — but it is the one row where a regression would be catastrophic and silent. Do **not** provoke an abort deliberately this sitting |
| H4.9 | minimum commanded/measured `pos` on any toss, and after any retract | `>= 0.0` rev — encoder zero is the excursion clamp's FLOOR and the host's own declared floor for this axis | `< 0.0` rev ⇒ **hard abort.** The bottom hard stop is at −0.1 rev (the axis homes downward into it) and the floor carries no margin for the position loop's +0.186 rev undershoot, so any commanded value below zero is planned travel onto the stop. Route to Phase 4's `SMOOTH_MOVE_POS_FLOOR_REV` |
| H4.10 | duration of any commanded hand move (last sample − first, from the trace) | `<= 0.8005` s | `>` that ⇒ the firmware's duration cap did not run (flash suspect). The cap is what keeps `catch_coordinator._PRIME_INFLIGHT_S = 1.2` s covering every profile the Teensy can emit; above it a re-prime tick can land inside a live ascent, which is the 2026-07-23 stutter |

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
limit: the shipped throw profile itself commands 1908 rev/s² at a 0.80 s flight
and 6055 rev/s² at the band top. **Whether to give `makeSmoothMove` a second,
higher arrest limit is an envelope decision, not an implementation one** — it
changes what the machine can physically do at the bench. Nothing in this sitting
requires the answer; the fallback is today's behaviour, so declining to decide
costs nothing.

---

## Section FW — `hand-command-continuity` Phase 6 (Platform Teensy `FW_VERSION`)

**Plan**: `plans/active/hand-command-continuity.md` § Phase 6
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
| **OK** | `1` | `PLATFORM_FW_CHECK: OK` | flashed, current — continue |
| **FAIL, pre-versioning** | `0 (PRE-VERSIONING)` | `PLATFORM_FW_CHECK: FAIL … PRE-VERSIONING` | the board answered and **has not been flashed** — flash, relaunch, re-check |
| **FAIL, other release** | the number | `PLATFORM_FW_CHECK: FAIL … v<n>` | flashed from a different tree — `git pull`, re-flash |
| **UNKNOWN** | `unknown` | `PLATFORM_FW_CHECK: UNKNOWN` | **not** a stale flash: no RobotState read landed at all. Most often the **known benign boot-read transient** on a launch-only restart (the same miss that gives you a surprise re-home) — **relaunch once and re-read**; investigate CAN3/relay only if it repeats. Note the verdict does **not** self-heal within a launch: it is re-read only on a UDP reconnect or a CAN3 WARN/BUS_OFF→OK edge, neither of which a clean launch produces |
| **(no verdict)** | key absent | *(no line)* | **not a board state** — the running node predates the check, i.e. `colcon build --packages-select jugglebot` was skipped. Rebuild both packages, source, relaunch. Never read an absent `FAIL` as a pass |

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

`Teensy_code/platformio.ini` builds the whole sketch for the Teensy 4.0
(`cd ros_ws/src/jugglebot/Teensy_code && pio run`). Before 2026-07-27 nothing in
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

**Plan**: `plans/active/levelling-frame-contract.md` § Phases 1–2
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
changed). **No firmware flash. No interface change** — `colcon build
--packages-select jugglebot` is enough.

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
>    Removing the swing is `plans/active/catch-reach-degenerate-overshoot.md`.
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
- **REPORT**: judge the catch **by eye** as well — tracker verdicts still read
  MISSED on real catches (the Phase-7 reload arc).
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
--packages-select jugglebot` **and a relaunch** (the launch runs the *installed*
copy). **No firmware flash** — nothing in this plan touches the Teensy.

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

Recording: add these to the § Recording topic list for any sitting you intend to
score — `/trajectory/status /trajectory/diagnostics /trajectory/target_feedback
/catch/dynamic_target /gravity_offset /throw_announcements` (the base list
already carries `/leg_setpoint_echo`, which is the commanded-pose source).

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

1. **Reboot the can-bridge Teensy** before the session (standing session rule).
2. `level` is per-boot — a manual `level` is **always** required first.
3. Record with the § Recording list **plus** the six catch topics above.
4. Score with CATCH-1 → CATCH-2 → CATCH-3, then
   `tools/probes/levelling_tilt_bag_check.py --offset <TILT_X> <TILT_Y> --t0
   <after the first go_home>` for the park (§ CHECK LVL-3's instrument).
5. Judge catches **by eye** as well as by `outcome` — tracker verdicts still read
   MISSED on real catches.

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
--packages-select jugglebot && source install/setup.bash`, then **relaunch** —
the launch runs the *installed* copy, and this change is in `trajectory_node.py`
and `motion/trajectory/planner.py`. **No firmware flash**; nothing here touches
the Teensy. Standing session rules still apply: reboot the can-bridge Teensy
first, and **check** `gravity_correction_loaded` — `level` only if it reads
`false` (standing rule 2, corrected 2026-07-27).

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

Validates: C-CATCH-1 on the self-toss path. Run a normal self-toss goal with the
§ Recording list **plus** `/trajectory/status /trajectory/diagnostics
/trajectory/target_feedback /catch/dynamic_target /gravity_offset
/throw_announcements`.

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

The tracker still reports `MISSED` on real catches, so **judge catches by eye as
well as by `outcome`** — as everywhere else in this file.

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
> prints `MANUFACTURES NOTHING (default 0.0 since 2026-07-26 — the bound is dormant,
> not removed)`, and the `0.20004 rad/s` bound itself is still computed and still
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
  `plans/active/catch-reach-degenerate-overshoot.md` Phase 2 /
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

1. Was the build actually installed? `colcon build --packages-select jugglebot`
   **and** a relaunch — § Build gate. A stale installed copy reproduces the
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

**Plan**: `plans/active/levelling-frame-contract.md` § Phase 3
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
> `TrajectoryStatus.msg` gained a field, so the § Build gate's
> `--packages-select jugglebot` is **not enough**:
>
> ```bash
> cd ~/Desktop/Jugglebot/ros_ws
> colcon build --packages-select jugglebot_interfaces jugglebot
> source install/setup.bash
> ```
>
> then **relaunch** `jugglebot_launch.py`. No firmware flash, no config
> regeneration. Also add `/trajectory/status` to the § Recording bag before
> LG-1 (it is in the shared list as of 2026-07-26 — check yours) or LG-4's
> diagnostic cannot be run at the end of the sitting.
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
>    to `level`.** `levelling_complete` is "since the last Teensy bootup" and the
>    § Shared preconditions power-cycle the can-bridge Teensy every sitting, so
>    it is `false` at every launch and the orchestrator's persisted auto-push
>    never fires first. **In practice every session genuinely needs a manual
>    `level`** — that has always been true; it is now enforced.
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
  proceeds. Then the can-bridge Teensy was **not** power-cycled this sitting, the
  orchestrator's persisted auto-push fired at the first IDLE, and a correction is
  genuinely loaded — the gate is right to pass. Power-cycle the Teensy per
  § Shared preconditions and re-run LG-1; do not score this as a failure, and do
  not score it as a pass of the gate either.
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

From the LG-2 state (levelled, toss accepted), **relaunch `jugglebot_launch.py`
without re-levelling**. A relaunch blanks `control_mode`, so you must re-arm
before the goal means anything:

**relaunch → `activate` → `trajectory` → read both flags → send the goal.**

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
  sitting for this to work — it is in the shared § Recording list as of
  2026-07-26; confirm yours has it *before* LG-1, because this measurement
  cannot be reconstructed afterwards.
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
- **ABORT**: `loaded-flips` shows `True` before the `level`, with the Teensy
  power-cycled per § Shared preconditions. Something republished a stale
  correction; the gate is then reporting a frame nobody established this
  session.
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
--packages-select jugglebot && source install/setup.bash`, then **relaunch** — the
launch runs the *installed* copy, and the change is in
`motion/trajectory/planner.py`. **No firmware flash. No config regeneration.**
Standing session rules still apply: power-cycle the can-bridge Teensy first, and
**check** `gravity_correction_loaded` after the relaunch — `level` only if it reads
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
looking for). Record with the § Recording list **plus** `/trajectory/diagnostics
/trajectory/target_feedback /catch/dynamic_target /gravity_offset
/throw_announcements`.

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
redesign: raise `planner._CATCH_TILT_THROUGH_RATE_RADPS` off zero and re-run.
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

The **C-CATCH-1 COUNTERFACTUAL** block now prints
`MANUFACTURES NOTHING (default 0.0 since 2026-07-26 — the bound is dormant, not
removed)` on its `arrival-rate bound` line. On a *post-fix* capture the recorded
reach and the counterfactual should collapse together.

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
  opens, and it needs its own plan.
