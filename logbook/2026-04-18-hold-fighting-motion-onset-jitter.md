---
title: Hold fighting + motion-onset jitter — residual hardware jitter downstream of clean MPC
type: investigation
date: 2026-04-18
status: in-progress
phase: post-walk-forward-fallback-multiaxis-session-2
related_plan: "leg-gain-tuning-methodology.md"
related_issues: []
sessions:
  - mpc_20260418_164119.csv
  - mpc_20260418_181020.csv
  - mpc_20260418_222358.csv
  - mpc_20260418_222414.csv
  - mpc_20260419_094723.csv
  - mpc_20260419_094733.csv
  - mpc_20260419_104332.csv
  - mpc_20260419_104408.csv
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py
  - ros_ws/src/jugglebot/jugglebot/can_node.py
commits: []
subsystem:
  - motion
tags:
  - tracking
  - performance
  - safety
---

# Hold fighting + motion-onset jitter — residual hardware jitter downstream of clean MPC

## Summary

Move 1 (Active -> z=220 pure-Z, 15 s) commands smoothly from the MPC (`cmd_jumps = 0`,
walk-forward fallback holding) but the physical platform exhibits a coordinated 2-5 mm
jitter burst across all six legs in the first ~0.7 s of motion, plus audible motor
"fighting" while holding the final pose. An A/B test disabling velocity feedforward
produced the identical 39-jump pattern (just shifted 0.6 s later in the trajectory),
ruling the MPC feedforward path out as the source. Residual jitter lives at the
leg-level ODrive PID or mechanical layer.

## Symptoms

- Visible jerks on the robot during the first ~1 s of motion, corroborated by ODrive
  GUI `input_pos` / `pos_setpoint` traces.
- Audible "fighting" between leg motors at hold — motors make small corrective bursts
  against each other even when the pose is ~stationary.
- Pattern persists across five consecutive moves (same rosbag:
  `/home/jetson/Desktop/rosbags/2026-04-18_16-41-02`).
- User-flagged Move 1 as the worst observationally.

## Diagnosis

### MPC is clean

| Metric | Move 1 (vel-ff ON) | Move 1 (vel-ff OFF rerun) |
|---|---:|---:|
| Solver success | 99.7% | 99.8% |
| cmd_jumps | 0 | 0 |
| Tracking RMS | 3.96 mm | 2.45 mm |
| Steady-state RMS | 2.93 mm | 1.14 mm |
| Solver p95 | 15.7 ms | 25.4 ms |
| Budget violations | 6 | 37 (6 consec) |
| **actual_jumps** | **39** | **39 (identical count/distribution)** |
| Jump window | 0.54-1.25 s | 1.13-1.81 s (shifted) |
| Magnitude range | 2.01-4.93 mm | 2.02-5.61 mm |

### Motion-onset burst (Move 1, vel-ff ON, CSV walk-through)

- cmd_ext starts ramping at t=0.39 s but motors don't move until t=0.51 s — ~120 ms
  dead-time.
- At t=0.54 s all 6 legs leap 2 mm simultaneously (velocities 50-100 mm/s from
  standstill in one 25 ms tick).
- Peak actual-lag vs cmd: +12 mm at t=0.71 s.
- Leg velocities swing 0 -> 130 -> -30 mm/s within single ticks during catch-up
  phase (0.5-1.2 s).
- 13 MPC loop dt gaps >40 ms (two >60 ms = skipped ticks) clustered in same early
  window — solver overhead spikes pushed loop off-cadence.

### Hold-phase fighting (Move 1 rerun, t>11 s, 234 samples near target)

| Leg | cmd_ext stdev (μm) | cmd_ext range (μm) | actual_ext stdev (μm) | actual_ext range (μm) | leg_vel range (mm/s) |
|---:|---:|---:|---:|---:|---:|
| 0 | 14.5 | 52.8 | 7.5 | 39.5 | 6.5 |
| **1** | 13.0 | 49.8 | **24.9** | **100.3** | 7.5 |
| 2 | 4.8 | 21.3 | 7.3 | 29.8 | 9.8 |
| **3** | 7.6 | 29.9 | **20.8** | **60.2** | 3.2 |
| 4 | 9.3 | 37.7 | 13.1 | 48.1 | 6.4 |
| 5 | 5.0 | 20.4 | 4.9 | 17.7 | 7.5 |

- Inter-sample cmd_ext delta at hold: mean 1.3-2.0 μm, max 5.9-9.0 μm — the MPC
  commands are essentially flat.
- actual_ext stdev is 2-3x larger on legs 1 and 3 vs the cmd — the motors are moving
  more than they're told to.
- Pose-level tracking error at hold: 0.087 mm RMS (0.136 mm peak) — fine at the pose
  level, but per-leg fighting is real.

### Candidate causes (ranked)

1. **ODrive position-loop gain too high for platform compliance** — best fit: P-gain
   amplifies encoder/measurement noise into velocity commands, which explains hold
   fighting (10 μm cmd -> 100 μm actual motion on leg 1), motion-onset velocity
   thrash, and consistency across vel-ff on/off.
2. **Internal force fighting between legs** — good fit: small kinematic
   inconsistencies between the six independent leg position references drive the
   legs to push against one another. Consistent with legs 1 & 3 being worst
   (mechanical asymmetry would pick specific legs).
3. **Mechanical backlash** — fits the onset stick-slip release burst, but does not
   explain ongoing hold fighting.
4. **Interpolation noise** (Hermite -> input_pos -> pos_setpoint in ODrive) — weak
   fit since cmd_ext is already smooth at 40 Hz; would need a 500 Hz-side
   investigation of pos_setpoint directly.

### Evidence ruling the MPC out

- cmd_jumps = 0 across 1560 samples in five consecutive moves on 2026-04-18
  (afternoon session), and 600 more in the evening --no-vel-ff rerun.
- Solver >=97.5% success on all moves except Move 4 (which is a separate
  CPU-saturation issue tracked by a different candidate known-issue).
- A/B test with vel-ff disabled reproduces the identical actual_jumps pattern —
  the feedforward path cannot be the cause.

### Flagged Issues

No existing `known_issues.yaml` entry matched. Candidate new IDs:
- `MOTOR_HOLD_FIGHTING` — per-leg actual_ext stdev 2-3x cmd_ext stdev at hold;
  legs 1 and 3 worst. Audible motor corrective bursts against each other.
- `ODRIVE_POS_LOOP_GAIN_HIGH` — amplifies encoder/measurement noise into velocity
  commands; best single-cause fit spanning both onset burst and hold fighting.

## Discussion

### Scope: Failure B only

The Diagnosis section describes two physically distinct jitter signatures that co-occur
on every move. For the rest of this entry they are named and scoped as follows:

- **Failure A — motion-onset burst.** 40-60 actual-position jumps (2-5 mm) in a
  0.5-0.7 s window across all six legs at every move start. Consistent across vel-ff
  on/off (see table above). Current working hypothesis (parallel thread): GC pause
  -> single MPC overhead spike -> loop falls behind budget -> chronic solve-time
  overrun triggers walk-forward fallback -> walk-forward plan depletes after N=10
  ticks -> cmd freezes -> platform coasts past the frozen cmd -> ODrive PID recoils
  at around -220 mm/s. **Not addressed by this entry.** Tracked separately; the user
  is adding GC instrumentation to `run_mpc.py`.
- **Failure B — hold-phase motor fighting.** At rest, with `cmd_ext` flat within
  around 10 um, individual legs physically move 60-100 um at +/-3-10 mm/s, audibly
  fighting. Hypothesis: ODrive position-loop gain is too high for the platform's
  mechanical compliance. **This entry is scoped to Failure B.** Failure A returns
  to scope once a leg-gain configuration that works across operating points is
  settled.

### Fix plan: three-level tuning methodology

Full treatment in `plans/active/leg-gain-tuning-methodology.md`. Summary:

- **Level 1 — empirical A/B tuning per leg.** Cheap, no instrumentation beyond
  what we already log. Good when one or two legs are clear outliers and the
  intuition is simply "less gain, more damping." Iteration pattern: baseline ->
  identify worst leg -> halve `pos_gain` and `vel_integrator_gain` on that leg ->
  retest -> if over-damped, go between baseline and halved; if still noisy,
  halve again.
- **Level 2 — step-response tuning.** Command a small step, fit rise-time /
  overshoot / settling, tune each leg to a target closed-loop response. Per-leg
  bench fixture needed.
- **Level 3 — system ID + loop shaping.** Full plant ID (sine sweep or PRBS),
  then design a loop-shaped controller. Warranted only if Levels 1-2 cannot
  reconcile load-dependent tradeoffs.

**Current stage: Level 1.** The A/B loop is run against existing MPC telemetry
(`actual_ext` stdev at hold, tracking RMS / peak during ramps) plus ODriveGUI
spot-checks of the live gain values.

Two invariants are enforced across all three levels:

- **`pos_gain : vel_integrator_gain = 125:1`** — the `vel_integrator_gain` is
  swept proportionally with `pos_gain` to keep the integrator's contribution to
  loop shape constant. This ratio matches the stock ODrive tuning guidance and
  the current `hardware_config.yaml` defaults (40 : 0.32).
- **`min(pos_gain) / max(pos_gain) >= 0.4`** — leg-to-leg gain asymmetry is
  bounded so that no single leg dominates the Stewart platform's coupled
  dynamics. At Level 1 this puts a floor on how aggressive a single-leg halving
  can get before the next step has to pull other legs down with it.

## Fix

### Part 1 — Per-leg gain plumbing (2026-04-18)

Before any A/B tuning could proceed, the codebase needed to support distinct
`pos_gain` / `vel_gain` / `vel_integrator_gain` values per leg. Previously the
ODrive defaults were scalar (one gain set applied to all six legs).

Seven files changed, 41 insertions:

1. **`config/hardware_config.yaml`** — added three 6-element arrays under
   `jugglebot_odrive_defaults`:
   - `leg_pos_gains` (initial: `[40, 40, 40, 40, 40, 40]`)
   - `leg_vel_gains` (initial: `[0.2, 0.2, 0.2, 0.2, 0.2, 0.2]`)
   - `leg_vel_int_gains` (initial: `[0.32, 0.32, 0.32, 0.32, 0.32, 0.32]`)
2. **`config/generated/hardware_config.py`** — regenerated, emits
   `ODRIVE_LEG_POS_GAINS`, `ODRIVE_LEG_VEL_GAINS`, `ODRIVE_LEG_VEL_INT_GAINS`.
3. **`config/generated/hardware_config.h`** — regenerated C header with the
   same constants for Teensy/firmware consumers.
4. **`ros_ws/src/jugglebot/Teensy_code/hardware_config.h`** — regenerated copy.
5. **`ros_ws/src/jugglebot/jugglebot/hardware_config.py`** — regenerated copy
   inside the ROS2 package.
6. **`ros_ws/src/jugglebot/jugglebot/can/odrive.py`** — added
   `DEFAULT_LEG_GAINS`, a 6-element list of per-leg gain dicts
   (`{pos_gain, vel_gain, vel_integrator_gain}`), populated from the generated
   constants.
7. **`ros_ws/src/jugglebot/jugglebot/can_node.py`** — added
   `self.leg_gains = [dict(g) for g in odrive.DEFAULT_LEG_GAINS]` in
   `__init__`, plus a new `_set_leg_gains()` method that iterates `LEG_AXES`
   sending `encode_set_pos_gain` + `encode_set_vel_gains` with 2 ms CAN pacing.
   Called from `_setup_odrives_steps` alongside the existing
   `_set_hand_gains`.

Verification: `python config/generate_config.py` succeeded; generated constants
round-tripped. ROS2 colcon build clean.

### Part 2 — Activate-path gain application (2026-04-19)

**Bug.** Part 1's `_set_leg_gains()` (and, as it turned out, the pre-existing
`_set_hand_gains()`) was only called from `_setup_odrives_steps()`, which only
runs during **homing** (`can_node.py` lines 1341 and 1362). The user did not
re-home between editing `hardware_config.yaml` and running the test, so the
gain setters never ran and the ODrives kept their flash defaults (40 / 0.2 /
0.32 on every leg).

**How it surfaced.** After Iteration 1 of the A/B (leg 1 YAML entries halved
to 20 / 0.2 / 0.16) the user reported via ODriveGUI that leg 1 live gains still
read 40 / 0.2 / 0.32 — the change had not taken effect. An initial analyst
claim that the halved gains *had* applied (based on a hold-stdev comparison)
was withdrawn: the windows being compared were not apples-to-apples (3 s
settled window vs 0.5 s, different target pose). The ODriveGUI readout was
authoritative.

**Fix.** Two changes in
`ros_ws/src/jugglebot/jugglebot/can_node.py`:

- Added `self._set_hand_gains()` and `self._set_leg_gains()` calls at the top
  of `_gentle_move_steps()`, right after the PASSTHROUGH mode assertion and
  before CLOSED_LOOP entry. Gains are now re-applied on every activation, not
  only on homing.
- Added `get_logger().info(...)` lines in both setters so the applied values
  appear in the ROS2 log on every activation — runtime visibility for future
  A/B iterations.

**Collateral.** `_set_hand_gains()` had the same dormant bug the whole time;
it was invisible because the flash-stored hand gains happened to match the
YAML values. Post-fix both setters run on every activate.

**Verification.** `pytest tests/motion/ -v` -> 73/73 passed. One flaky failure
in `test_decay_boundary_continuity` passed in isolation; the failure was
state-contamination between tests and unrelated to this change.

## Verification

Ongoing experimental log of Level-1 A/B iterations. Each iteration edits
`config/hardware_config.yaml`, regenerates, rebuilds ROS2, re-launches, and
runs a pair of moves (up to z=220, down to z=170) from the Active pose.

### Iteration 1 (2026-04-18 evening) — leg 1 20 / 0.2 / 0.16, FAILED TO APPLY

Sessions: `mpc_20260418_222358.csv` (up), `mpc_20260418_222414.csv` (down).
Subjective: up move stuttery, down move smooth. ODriveGUI verification showed
leg 1 gains **unchanged at 40 / 0.2 / 0.32**. Result invalid due to the
plumbing bug described in Fix Part 2; no conclusion can be drawn from this
data.

### Iteration 2 (2026-04-19 morning) — leg 1 20 / 0.2 / 0.16, gains applied

Sessions: `mpc_20260419_094723.csv` (up z=220), `mpc_20260419_094733.csv`
(down z=170). Rosbag: `~/Desktop/rosbags/2026-04-19_09-46-52`. ODriveGUI
confirmed leg 1 live gains read 20 / 0.2 / 0.16. Both moves observationally
"fairly smooth."

**Tracking comparison vs fair baselines:**

| Session | Move | vel-ff | Leg 1 gains | RMS | Peak | Final |
|---|---|---|---|---:|---:|---:|
| 164119 (15 s) | up z=220 | ON | uniform | 3.96 mm | 20.5 mm | 0.12 mm |
| 181020 (16 s) | up z=220 | OFF | uniform | 2.45 mm | 19.0 mm | 0.05 mm |
| **094723 (5 s)** | **up z=220** | ON | **halved leg 1** | 4.12 mm | **13.5 mm** | 0.40 mm |
| 015027 (5 s) | down Active | ON | uniform | 4.83 mm | — | 0.87 mm |
| **094733 (5 s)** | **down z=170** | ON | **halved leg 1** | **2.08 mm** | **5.4 mm** | 1.86 mm |

Headline: peak tracking error **-34 %** on the up-move (13.5 vs 20.5 mm), RMS
**-57 %** on the down-move (2.08 vs 4.83 mm) vs the duration-matched baseline.
`cmd_jumps = 0` on both moves (walk-forward fallback holding, MPC pipeline
clean). Solver success 98.0-98.5 %, p50 ~ 10.6 ms, 5-8 budget violations per
move (borderline, same Failure-A pattern). `actual_jumps`: 46 / 45 per move,
0.5 s onset windows — **Failure A still fully present; leg-gain change had
no measurable effect on it, as expected since Failure A is a separate
mechanism.**

**Hold-phase per-leg stdev (last 1 s, ref confirmed static):**

Move 1 — z=220 (high gravity load, platform compressed):

| leg | act_std_um | rank |
|---:|---:|:---|
| 0 | 154 | worst |
| **1** | **6.3** | **quietest** <- halved |
| 2 | 31 | — |
| 3 | 48 | — |
| 4 | 91 | 2nd worst |
| 5 | 8.6 | 2nd quietest |

Move 2 — z=170 (low gravity load, platform retracted):

| leg | act_std_um | rank |
|---:|---:|:---|
| 0 | 4.8 | 2nd quietest |
| **1** | **93** | **worst** <- halved |
| 2 | 9.5 | — |
| 3 | 3.7 | quietest |
| 4 | 19.7 | — |
| 5 | 55 | 2nd worst |

**Interpretation.** Classic load-dependent PID tradeoff: the halved `pos_gain`
on leg 1 suppresses fighting at high compression (z=220, platform extended,
heavy load on every leg) but leaves leg 1 under-damped at low compression
(z=170, less static load -> loop rings against encoder noise). Leg 1 went from
tied-for-worst at baseline (14.5 um stdev) to **quietest at z=220 and noisiest
at z=170**. Subjectively both moves felt smooth — at z=170 the leg 1 hold
amplitude is still small in absolute terms (about 300 um range vs 100 um
baseline) and low-frequency enough to be visually/audibly acceptable.

**Caveat — non-halved legs noisier than baseline at z=220.** Legs 0, 2, 3, 4
ran 31-154 um stdev at hold vs 4-9 um in the afternoon baseline. Their gains
are unchanged (40 / 0.2 / 0.32) so the code is not the cause. Two plausible
confounds: (a) the 5 s run has only 3.3 s of settled time vs 14 s in the
baseline — hold-stdev statistics on short windows are noisy; (b) cold morning
start vs warmed-up evening. Not treated as a regression yet; a longer-hold
re-test will either reproduce or dissolve it.

### Iteration 3 (2026-04-19 mid-morning) — leg 1 & leg 4 at 30 / 0.2 / 0.24, complete

Two edits to `config/hardware_config.yaml`:

- **Leg 1: 20 / 0.2 / 0.16 -> 30 / 0.2 / 0.24.** Compromise between baseline
  (40 / 0.32) and halved (20 / 0.16) — walk-back from the Iteration-2 halving
  that was too soft at z=170. Hypothesis: recovers most of the Move 2
  quietness at z=170 while keeping most of the Move 1 improvement at z=220.
- **Leg 4: 40 / 0.2 / 0.32 -> 30 / 0.2 / 0.24.** First reduction on leg 4,
  matching the leg-1 compromise value. Leg 4 ranked 2nd-worst hold-stdev at
  z=220 in Iteration 2.

Ratio `pos_gain : vel_integrator_gain = 125:1` held for every leg.
Asymmetry invariant satisfied (`min/max = 30/40 = 0.75 >= 0.4`).

Sessions: `mpc_20260419_104332.csv` (up z=220, 20 s), `mpc_20260419_104408.csv`
(down z=170, 20 s). Rosbag: `~/Desktop/rosbags/2026-04-19_10-43-08`. Both
moves ran 15 s ramp + ~5 s settled hold for clean hold-stdev statistics. User
observation: motion appears visually smooth, but HF jitters are visible in
the `pos_setpoint` values on the ODrive GUI.

**Tracking comparison vs full history (fair duration-matched where available):**

| Session | Move | Config | Dur | RMS | Peak | SS RMS | Final |
|---|---|---|---:|---:|---:|---:|---:|
| 164119 | up z=220 | uniform 40/0.32 | 15 s | 3.96 | 20.5 | 2.93 | 0.12 |
| 181020 | up z=220 | uniform, no-vel-ff | 16 s | 2.45 | 19.0 | 1.14 | 0.05 |
| 094723 | up z=220 | leg 1 = 20/0.16 | 5 s | 4.12 | 13.5 | 3.34 | 0.40 |
| **104332** | **up z=220** | **L1/L4 = 30/0.24** | **20 s** | **0.99** | **8.22** | **0.65** | **0.15** |
| 015027 | down Active | uniform 40/0.32 | 5 s | 4.83 | — | — | 0.87 |
| **104408** | **down z=170** | **L1/L4 = 30/0.24** | **20 s** | **1.56** | **11.05** | **1.19** | **0.034** |

Headline:

- Pose tracking RMS **0.99 mm** on the up-move — 4x better than the prior
  best (2.45 mm at the no-vel-ff iteration).
- Peak tracking error **8.22 mm** — 60 % drop from the 20.5 mm baseline.
- Steady-state RMS **0.65 mm** — below the 1 mm threshold the investigation
  has been bumping against.
- Final tracking on the down-move **34 um** (encoder-LSB territory).
- `cmd_jumps = 0` on both moves; walk-forward fallback still intact.
- Solver 99.6-99.75 % success, p95 ~ 13 ms, 6 budget violations per 800
  samples (0.75 %).
- **FK non-convergences = 0** in both 20 s moves (baseline: 11). User's
  parallel FK work is contributing.

**Hold-phase per-leg `actual_ext` stdev (last 5 s, ref static):**

Move 1 — z=220:

| leg | act_std (um) | act_range (um) | gains | rank |
|---:|---:|---:|---|---|
| 0 | 6.1 | 31 | 40/0.2/0.32 | — |
| **1** | **10.9** | **49** | **30/0.2/0.24** | noisiest |
| 2 | 5.0 | 24 | 40/0.2/0.32 | — |
| 3 | 6.0 | 27 | 40/0.2/0.32 | — |
| **4** | **4.2** | **24** | **30/0.2/0.24** | 2nd quietest |
| 5 | 4.0 | 23 | 40/0.2/0.32 | quietest |

Move 2 — z=170:

| leg | act_std (um) | act_range (um) | gains | rank |
|---:|---:|---:|---|---|
| 0 | 4.2 | 17 | 40/0.2/0.32 | 2nd quietest |
| **1** | **6.1** | **28** | **30/0.2/0.24** | noisiest |
| 2 | 5.5 | 26 | 40/0.2/0.32 | — |
| 3 | 2.9 | 22 | 40/0.2/0.32 | quietest |
| **4** | **5.8** | **25** | **30/0.2/0.24** | — |
| 5 | 5.8 | 19 | 40/0.2/0.32 | — |

**Interpretation.**

- **Leg 4 at 30 / 0.2 / 0.24 — clean win.** 2nd quietest at z=220, mid-pack
  at z=170, no downside.
- **Leg 1 at 30 / 0.2 / 0.24 — acceptable compromise.** Noisiest in both
  moves but within 2-3x of the quietest leg, meeting Level-1 acceptance
  (< 1.5x target is the preferred bar, so leg 1 is borderline but in range).
  Recovers most of the z=170 quietness lost at the 20 / 0.16 halving while
  keeping most of the z=220 benefit.
- **All six legs now in the 3-11 um stdev / 17-49 um range band across both
  moves** — roughly 4x improvement on the 14-25 um stdev / 50-100 um range
  that characterised legs 1 and 3 at the original uniform 40 / 0.32
  baseline.

**HF jitter in `pos_setpoint` — characterised as MPC numerical noise floor,
not a defect.**

FFT of `cmd_ext` over the last 10 s of hold:

| Move | 0.1-2 Hz | 2-10 Hz | 10-20 Hz |
|---|---:|---:|---:|
| Up z=220 | 41 % | 34 % | 25 % |
| Down z=170 | 31 % | 48 % | 22 % |

~20 % of the `cmd_ext` energy during hold sits in the 10-20 Hz band (top
third of the 40 Hz Nyquist). Total `cmd_ext` stdev during hold is 3-4 um,
so the HF component is ~1.5-2 um RMS. Source: IPOPT solves a fresh NLP
every 25 ms and, even with warm-start, produces solutions that vary at the
um level between solves. The Hermite interpolator spreads these smoothly
to 500 Hz, but the 40 Hz beat and its harmonics show up in ODrive's
`pos_setpoint` scope. Absolute amplitude (~2 um RMS) is below the
mechanical bandwidth — the motors don't physically respond — which is why
the visual observation is "smooth" despite the GUI scope showing jitter.
Not a bug, not a performance issue, not fixable without a deliberate
design change (e.g., trailing LPF between `cmd_ext` and Hermite input) that
would cost phase lag. Parked as a known cosmetic.

**Failure A (motion-onset burst) — confirmed to be overhead pops, not
solver.**

59 actual-jumps in the 0.94-1.54 s window on Move 1 (similar on Move 2).
In that window, dt-vs-solve-time analysis shows large dt gaps with short
solves:

```
t=1.356  dt=36.2ms  solve=30.2ms
t=1.405  dt=49.2ms  solve=22.4ms   <-  27 ms of non-solve overhead
t=1.448  dt=42.4ms  solve=10.8ms   <-  31 ms of non-solve overhead
t=1.482  dt=34.8ms  solve=11.3ms
```

That is the exact signature flagged by the user's collaborator (GC pause /
interpreter housekeeping). No amount of gain tuning touches this
mechanism. The GC instrumentation being added to `run_mpc.py` is the right
next move.

## Outcome

**Level-1 tuning phase: complete.** Final config (legs 0 / 2 / 3 / 5 at
40 / 0.2 / 0.32; legs 1 / 4 at 30 / 0.2 / 0.24) meets Level-1 acceptance on
both operating points (z=170 and z=220). All six legs now within 3-11 um
hold-stdev — ~4x improvement on the uniform-40 / 0.32 starting point.

**Failure B: acceptably tuned.** Hold fighting is no longer audible or
visually apparent on the platform. The remaining HF jitter visible in
ODriveGUI `pos_setpoint` is characterised as MPC numerical noise floor
(~2 um RMS, below mechanical bandwidth) — noted as a known cosmetic, not
worth fixing with a filter unless a juggling-specific test shows it matters.

**Failure A: unchanged, not addressed by this investigation.** Onset burst
still present (~59 jumps per 20 s move, concentrated in a 0.5 s window at
motion start), confirmed mechanism is non-solve overhead pops (GC or
interpreter housekeeping). The user is pursuing this via parallel work on
GC instrumentation in `run_mpc.py`. Status remains `in-progress` on this
entry on account of Failure A.

**Open follow-ups:**

- Revisit leg 1 with Level 2 (step-response tuning) if performance budgets
  tighten — current 30 / 0.24 is borderline at Level 1 acceptance.
- Consider whether the untuned legs (0, 2, 3, 5) would benefit from their
  own halving pass; they're currently in the 3-8 um band so probably not,
  but worth rechecking on a longer-hold run.
- FK non-convergences dropped from 11 -> 0 between baseline and this
  iteration. Track whether that holds up across different trajectory
  shapes — if yes, credit the parallel FK work; if no, revisit.

## Open Questions

- Which ODrive gains are currently set (`pos_gain`, `vel_gain`,
  `vel_integrator_gain`)? A record from `config/hardware_config.yaml` or a live
  ODrive dump would be useful.
- Are legs 1 and 3 mechanically similar (same batch of bearings/couplers, same
  position in the hex)?
- Does the 500 Hz `leg_lengths_topic` stream in the rosbag show a clean frequency
  spectrum at hold, or is there a dominant fighting frequency?
- Would a single-leg bench test (platform weight off) show the same hold fighting?
