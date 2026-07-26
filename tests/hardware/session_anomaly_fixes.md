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
  /link_status /rosout /trajectory/status
```

Note the bag directory name — the analysis commands below take it as `--bag`.

`/trajectory/status` was added to this list on 2026-07-26 (extra topics are
harmless to every existing analysis command). § Section LVLGATE's CHECK LG-4
diagnoses a suspected staleness false-positive from its inter-arrival gaps, and
that measurement is only obtainable *after the fact* if the topic was in the bag
from the start — it is 5 Hz, so the cost is negligible.

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
LOG=~/.ros/log/$(ls -t ~/.ros/log | head -1)/launch.log
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
first, and `level` is per-boot so a manual `level` is always required.

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

> **⚠ SUPERSEDED 2026-07-26 by § Section ZSEAT — four of the rows below now ABORT
> on CORRECT behaviour.** The operator set the manufactured seat rate
> (`planner._CATCH_TILT_THROUGH_RATE_RADPS`) to **0.0**, so the reload's
> through-seat no longer exists at all. Corrections, row by row, so a top-to-bottom
> reader cannot act on the stale table:
>
> | row | table below says | **actually correct now** |
> |---|---|---|
> | segment count | `3 → 3`, ABORT if 2 | **2** — reach + quiescent hold. 3 would mean the zero default did not land |
> | seat rate at ball contact | `0.070000` unchanged, ABORT below `0.0665` | **`0.000000 rad/s`** — a parked rim is the intended state |
> | settle `rx` / `ry` | `+1.844635 / −10.928741°` | **`+1.774062 / −10.636334°`** = exactly the commanded target (a further `−0.070573 / +0.292407°`) |
> | predicted acc / jerk | `142.0 / 3935` | **`37.9 / 170`** (and vel `23.8 → 29.0` — it goes *up*; see ZSEAT-3) |
> | the "PASS / ABORT" paragraph under the table | ABORT if the last 0.8 s is **flat** | **flat is the PASS** — inverted outright |
>
> The rest of the table — the aim rotation, `peak off the park`, the
> `N further catch install(s)` census line — is unaffected and still reads as
> written. Score the reload against § Section ZSEAT, not against this table.

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
> 2. **Re-`level` after every relaunch, including a mid-sitting one.** The
>    correction lives in `trajectory_node`'s memory, not on the Teensy.
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
> It supersedes four rows of **§ CHECK CCATCH-3** — a banner at the head of that
> check lists them, and one of them (*flat commanded tilt in the last 0.8 s*) is
> **inverted**: what CCATCH-3 tells you to ABORT on is now the PASS. Read that
> banner before scoring any reload.
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
`level` after every relaunch.

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
| caught / attempted | **≥ 12/19 (0.63)** — i.e. no worse than sitting 4's 15/19 (0.79) by more than one binomial sigma (σ ≈ 0.09 at n = 19) | **≤ 11/19 (0.58)**, i.e. ≥ 2σ down |
| **bounce-outs** (touched the cup, left it) | **≤ 1** across the sitting | **≥ 3**, or **≥ 2 consecutive** — this is the seat-deflection signature and it is what the `0.07` existed to prevent |
| bounce-out vs sitting 4 | sitting 4's misses were BB scatter + late arrival, **not** bounce-out | any bounce-out cluster that was **absent** before is attributable to this change until shown otherwise |
| commanded tilt over the last 0.8 s before landing (FK of `/leg_setpoint_echo`) | **flat**, `< 0.05°` of motion — the rim is parked, as intended | `≈ 0.9°` of round trip — the zero default did **not** land (stale install: colcon + relaunch) |
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
| commanded platform pose (FK of `/leg_setpoint_echo`) over `release ± 0.10 s` | **flat**, `< 0.02°` and `< 0.2 mm` of motion | any commanded motion — a plan is running through the release |
| `/trajectory/status` `plan_time_remaining_s` at release | ≤ 0 (terminal hold), or a hold segment | a `move` plan mid-reach |
| Tier 8b only: first `catch/dynamic_target` timestamp | **≥** `t_release` | before `t_release` — the deferred reach fired early |

### Not in this section

- Whether the *level* (self-toss) catch commands a swing — § CHECK CCATCH-2,
  unchanged by this section.
- Whether the seat's **aim** is right — § CHECK CCATCH-3's aim rotation row, still
  valid.
- Re-tuning the seat rate. That is the follow-on session ZSEAT-2's ABORT path
  opens, and it needs its own plan.
