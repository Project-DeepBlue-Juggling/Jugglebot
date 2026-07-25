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


---

## Section LVL — `levelling-frame-contract` Phases 1–2 (one levelling frame)

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
> 3. **THE PRE-THROW TILT SWING IS STILL THERE. This fix does not remove it.**
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
while `RobotState.levelling_complete` still reads True (contract § Known hazard;
`levelling-frame-contract` Phase 3 owns the closure).

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

From the same bag, read `/rigid_body_poses` for the `Platform` body.

- **PASS**: between goals (during the park plateau LVL-3 identified), the
  Platform's total tilt against the `Base` frame is within **±0.10°** of level —
  i.e. it does **not** track the commanded `−0.78°`, because the correction's
  whole purpose is that a commanded `−0.78°` *is* physically level.
- **ABORT**: the platform is physically tilted by ~0.78° while parked. That
  inverts the sign of the correction: `correction_from_offset` negates the
  measured error, and a physical tilt of the same magnitude means it is being
  **added** instead. Stop the sitting.
- **REPORT**: the Platform tilt during the catch settle, expected **0.30°** off
  level (the through-seat overshoot).
- **REPORT**: the Platform's peak tilt during the catch reach, expected
  **≈2.92°** off level at a 3.70 s catch lead (`0.789132° × lead`). This is the
  mocap-side confirmation of pre-brief item 3 — the swing is still there, and it
  is *smaller* than the pre-fix `+3.0992°` by exactly the correction's
  contribution at the peak. Not a gate.

### CHECK LVL-5 — catch error (expectation-setting, not a gate this round)

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

- **`REJECTED_NOT_LEVELLED`** is `levelling-frame-contract` **Phase 3** and has
  not landed. A toss commanded before `level` is still accepted today; do not
  score it.
- The **hand dip** is `hand-command-continuity` (§ Section HAND). If both have
  landed, validate them in one sitting but score them separately.
