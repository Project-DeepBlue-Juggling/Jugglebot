# Hardware Session — 2026-07-25 self-toss anomaly fixes: bench validation

**Run**: the 2026-07-25 self-toss anomaly-fix run (five sibling plans, one phase
runner). Each plan appends its own checks to this file **in the order the
operator will execute them** — append a new section, never rewrite or reorder
someone else's.

**Goal**: confirm on hardware that each landed anomaly fix does what its plan
claims, with numeric PASS/ABORT criteria and a named analysis command per check
so a failure routes straight back to the plan + phase that owns it.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command in this file.**
  The implementing sessions prepared the exact commands and criteria and verify
  read-only artefacts (logs, bags) afterwards.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.**
- E-STOP always in reach. Any ABORT criterion ⇒ stop, capture the bag and the
  node logs, and debrief before retrying.

## Shared preconditions (do these once per sitting)

- Jugglebot powered, ODrives up, CAN3 healthy; QTM streaming **Base +
  Platform**; `/rigid_body_poses` flowing.
- **POWER-CYCLE THE CAN-BRIDGE TEENSY** before the sitting. Tracking lag grows
  with Teensy uptime, so log `uptime_ms` alongside every timing measurement.
- `levelling_complete` is per-boot: run a manual `level` after every relaunch.
- `run_mpc.py` is **NOT** running unless a check says so (sole-binder :5557).

### Build gate

Every section below states its own build needs. Where a section says *colcon +
relaunch*, run:

```bash
cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select jugglebot
source install/setup.bash
```

then **relaunch** `jugglebot_launch.py` — the launch runs the *installed* copy,
so a relaunch without a rebuild keeps the old code.

### Recording (do this for every check that produces a verdict)

```bash
mkdir -p ~/Desktop/rosbags && cd ~/Desktop/rosbags
ros2 bag record -o "$(date +%Y-%m-%d_%H-%M-%S)" \
  /robot_state /leg_setpoint_echo /platform_target /rigid_body_poses \
  /link_status /rosout
```

Note the bag directory name — the analysis commands below take it as `--bag`.

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
  `fixed-shape branch` (four cases: `clean`, `overshoot`, `short-flight`,
  `braking-prelude`). Judge on the exit code and on both lines being `GATE
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
| 3 | `peak` | `<= 10.060` rev (`x3` 9.9594 + 0.10) | `> 10.060` rev; **hard abort at `> 10.5` rev** |
| 4 | `dip_below_x3` | `<= 0.100` rev (`<= 3.2` mm) — the row prints `OK` | `> 0.100` rev — the row prints `OVER`. Pre-fix range was **0.339–1.748 rev = 10.7–55.3 mm** |
| 5 | `pullback` | `>= -5.0` rev/s, **given row 3 passed** | `< -5.0` rev/s. Pre-fix range was **−17.9 to −42.4 rev/s** |
| 6 | `catch_desc` | present, within ~20 ms of `event − t_acc_catch` | absent ⇒ the catch never fired; check the Teensy serial for `Not enough time for smooth-move` |
| 7 | `first_neg_cmd` | equal to `catch_desc` (no annotation printed) | annotated `<-- NOT the catch descent (a brake?)`: **REPORT, do not abort.** Expected after Phase 4 lands (step 3 charters a braking prelude); before Phase 4 it means an unexplained downward command |

**Why row 4 and not `dip_bottom` / `dip`.** `dip` is peak-minus-bottom, so it is
non-zero on *any* capture that overshoots and settles — including a perfectly
fixed one (0.6 mm on the probe's own clean synthetic) and including the bounded
overshoot plan Phase 4 step 2 makes the *expected* behaviour (20.2 mm at
10.60 rev, i.e. the size of the smallest pre-fix defect). Gating on it would score
a working fix as FAILED. What separates the defect from a healthy stroke is the
*sign* of the excursion about the stroke end: pre-fix the position loop yanks the
hand **below** `x3`; a healthy stroke settles **onto** `x3` from above and never
goes under (the four synthetic post-fix shapes read 0.000–0.001 rev). Same reason
`pullback` is bounded rather than required non-negative: a healthy settle from the
coasting peak is genuinely negative — −0.31 rev/s at 0.02 rev of overshoot, −1.58
at the 10.060 rev ceiling of row 3, −10.03 at 10.60 rev.

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
