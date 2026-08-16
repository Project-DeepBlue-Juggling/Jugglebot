---
title: The retiming model's honest durations exceed the tracking envelope on lean traverses — deviation 0.45→0.73 rev on the shipped config; model shipped OFF until accel FF
type: investigation
date: 2026-07-17
status: resolved
phase: "MVP trajectory bringup — S4 working point on hardware (retime model first hardware A/B)"
related_plan: accel-ff-inertia.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/hardware/mvp_bench_runbook.md
  - logbook/2026-07-17-wobble-latch-unshaped-traverse.md
  - logbook/2026-07-17-shaped-planning-efficiency-implemented.md
commits:
  - 4e4545e
subsystem:
  - motion
  - config
tags:
  - retime-model
  - lean-shaping
  - tracking-lag
  - max-deviation
  - working-point
  - shaped-planning-efficiency
---

## Summary

The operator's second session of the day (bag `2026-07-17_19-32-03`) ran the
intended lean A/B: `--lean-gain 0.0` latched the guard on the ±150 x-traverse
again (1.076 rev — third reproduction of the morning's cliff, unchanged), and
`--lean-gain 0.6` completed but felt "smoother, but not nearly as smooth as
yesterday". Their two hypotheses — a compute bottleneck, or ringing/instability
introduced by the SPE rewrites — are both **killed by data** (max emit gap
28.8 ms all session; post-arrival velocity zero-crossings 1–2 both days, no
oscillation train). The real cause is the third option nobody padded a
hypothesis for: **the retiming model doing exactly what it was built to do.**

Yesterday's lean-0.6 moves were timed by the legacy stretch+bisection loop,
which overshoots hardest exactly where lean content is largest: the two ±150
traverses executed at peak leg jerk **12000 / 11534** of the 30000 budget
(everything else pegs ~26–29k). Today, the retiming model timed those same
traverses honestly — peak jerk **27406 / 27276**, +13 % peak vel, +52–62 %
peak accel, x-traverse duration 1.60 → 1.42 s — and the deviation went
**0.450 → 0.727 rev** (y: 0.289 → 0.557). Every *other* shaped move planned
near-identically to yesterday (z-dn 185.6/1219/28270 vs 188.0/1250/28979);
their deviation deltas (+6–80 % across the eight matched non-traverse moves,
median ~+43 % — noisy at small denominators, the biggest percentages sit on
the smallest deviations ≤ 0.28 rev) are plant-side, the same direction as the
sticky-day effect measured this morning at byte-identical unshaped plans. So: no ringing, no bottleneck —
the model removed duration padding that yesterday's validated smoothness was
silently built on, and at honest timing the shipped config's flagship move
rides **above the operator's ~0.6 rev in-move ABORT line**.

Fix shipped: `trajectory_op.retime_model: false` (YAML + regen). The legacy
loop on the batched 1600-mesh gate still plans shaped moves in ~0.23 s — 5×
faster than the pre-SPE ~1.2 s the operator originally flagged — and the flag
re-enables the model for any future A/B without code changes. Re-enable
condition: accel FF landing (raises the tracking envelope), or a
tracking-aware duration floor.

## Symptoms

- Operator: two battery runs, `--lean-gain 0.0` then `0.6`. "The 0.0 run was
  just as wobbly as before. The 0.6 run was smoother, but not nearly as smooth
  as yesterday. Either we're hitting some kind of compute limit/bottlenecking,
  or something in the latest batch of efficiency re-writes has introduced some
  kind of ringing or instability."
- Bag `19-32-03`: run 1 (lean 0.0) `ACTIVE → FAULT` at t=31.7 s, MAX_DEVIATION
  on the x-traverse at 1.076 rev — same move, same magnitude as the morning's
  two latches (`logbook/2026-07-17-wobble-latch-unshaped-traverse.md`). Run 2
  (lean 0.6) completed, no fault.

## Diagnosis

Hypotheses tested in order, each against the bag:

1. **Compute bottleneck — NO.** `max_emit_gap_ms` peaked at 28.0 (run 1) /
   28.8 ms (run 2) against the 25 ms nominal — indistinguishable from
   yesterday's clean sessions. Planning never starves the emitter (it plans
   while holding, by design).
2. **Ringing / instability — NO.** Post-arrival encoder velocity
   zero-crossings (|v| > 0.05 rev/s): today 2 / yesterday 1 on the x-traverse,
   today 2 / yesterday 0 on tilt⁻ — single catch-up excursions, no oscillation
   train. Peak post-arrival velocity identical across days (1.36 vs 1.40
   rev/s). Nothing rings; the excursion is just bigger because the deviation
   is bigger.
3. **Retiming model honest timing — YES.** Matched lean-0.6 moves, yesterday
   (sweep `22-06-30`, legacy-timed) vs today (`19-32-03`, model-timed):

   | move | vpk y→t (mm/s) | apk y→t (mm/s²) | jpk y→t (mm/s³) | dev y→t (rev) |
   |---|---|---|---|---|
   | z dn | 188.0 → 185.6 | 1250 → 1219 | 28979 → 28270 | 0.358 → 0.454 |
   | x+150 | 96.1 → 91.2 | 775 → 649 | 27556 → 27246 | 0.215 → 0.320 |
   | **x traverse** | 122.3 → **138.5** | 515 → **785** | **12000 → 27406** | **0.450 → 0.727** |
   | **y traverse** | 105.6 → **120.9** | 427 → **691** | **11534 → 27276** | **0.289 → 0.557** |
   | tilt⁻ | 249.4 → 245.4 | 1453 → 1407 | 29284 → 28270 | 0.568 → 0.664 |

   Non-traverse plans are near-identical, so their deviation deltas are
   plant-side by the same matched-plan logic as the morning's entry. The
   full set runs +6–80 % (median ~+43 %) vs the morning's +10–30 % on large
   unshaped moves — wider because these deviations are small (≤ 0.28 rev)
   and percentage deltas amplify at small denominators; same direction, and
   no dependence on plan changes. The
   traverses are *different plans*: legacy's `_stretch_factor` assumes 1/Tⁿ
   scaling while lean's tilt content scales ~1/T³, so bisection overshoots
   most on high-lateral-accel moves — the traverses ran at ~40 % of the jerk
   budget yesterday. The model prices them honestly (jerk-pegged like every
   other move, duration 1.60 → 1.42 s on x) and the velocity-loop tracking
   deficit does the rest. (Run 2's first move planned gentler than yesterday's
   — vpk 99.3 vs 173.8 — because its commanded stroke was 0.300 vs 0.655 rev:
   post-fault-recovery start pose, not a model anomaly.)

## Discussion

**The gate's invariant class does not include trackability — and legacy
overshoot was accidentally covering for that.** Every plan both days is
gate-feasible; the SPE work proved the model never bypasses the gate and never
returned an infeasible plan (zero verify rejections, 174 corpus cases). What
the corpus could not see is that at this working point the **binding
constraint on lean traverses is not the kinematic jerk/accel limits but the
velocity loop's tracking envelope** (~2.2–2.7 rev/s chase under coordinated
load, the 2026-07-16 mechanism). Yesterday's smoothness on those moves was
manufactured by solver slop: the legacy loop's overshoot — a defect by the
SPE plan's framing, measured at +1.2–14.7 % on its corpus — reaches ~2× in
jerk terms on real lean traverses (the corpus's move mix was milder than the
battery's ±300 mm sweeps). Remove the slop and the commands are correct per
the limits, but the limits themselves were calibrated *against padded
behaviour*: the operator validated a feel, not the numbers, and the numbers
were never what the hardware was actually executing on those moves.

**Why ship the flag OFF rather than retune?** Three alternatives were
considered. (a) Raising the model's inflation factor (1.02 → ~1.3) would
approximate legacy padding on traverses but also slow every move legacy
already timed honestly — z/tilt/singles pegged ~27–29k jerk yesterday and
their feel was validated at that speed; a global pad is worse fidelity to the
validated behaviour than the legacy loop it replaces. (b) Cutting the session
jerk limit (30000 → ~15000) same bluntness, plus it invalidates the S4
working-point record. (c) A lean-aware or tracking-aware duration floor is
the *principled* mechanism (plan durations against what the plant can track,
not only what kinematics allow) but is new design work — exactly what the
accel-FF chapter makes unnecessary if FF closes the tracking deficit. Flag
OFF restores the hardware-validated behaviour byte-for-byte on the timing
path, costs ~140 ms of planning (0.09 → 0.23 s, still 5× better than the
pre-SPE 1.2 s that motivated the work), reverses with one YAML flip (+ regen
+ `colcon build` + relaunch — the flag is config-gated, not runtime), and
keeps the model's code and tests intact for the re-enable.

**The class, one level up (K-contract lens):** "gate-feasible but
untrackable" is the same failure class as the morning's unshaped-traverse
cliff — the unshaped moves were *always* in this class (hence the latches),
and the model just moved the shaped traverses into it too. The whole class
closes when commanded acceleration is backed by feedforward torque
(`plans/parked/accel-ff-inertia.md`), or structurally via a tracking-aware
envelope in the planner. Until one of those lands, anything that shortens
durations — solver honesty included — converts tracking margin into
deviation. Worth remembering for the next "make planning faster/tighter"
idea: **duration slop was load-bearing.**

**What the corpus missed, for future test design:** the SPE optimality corpus
measured legacy overshoot at +1.2–14.7 % — true for its move mix, but the
battery's lean traverses sit far outside that tail (~36 % in duration-
equivalent terms). A corpus intended to bound "how much will behaviour change
on hardware" must include the extreme moves of the *actual operational
battery*, not a synthetic mix. (Adding the ±150 traverse family to the corpus
is cheap and would have predicted today's numbers offline.)

## Fix

- `config/hardware_config.yaml` `trajectory_op.retime_model: true → false`
  with the full rationale + re-enable condition in the comment; regenerated
  (`python config/generate_config.py`) → `JB_TRAJ_RETIME_MODEL = False` in
  both Python configs and the (firmware-unused) header constants. No code
  change: the flag was built for exactly this A/B.
- Runbook + prior entries corrected: the "~91 ms" shipped planning figure and
  the morning entry's "shipped config rides ≤ ~0.45 rev" margin claim were
  model-ON / legacy-timed figures respectively — both now annotated (shipped
  planning is ~0.23 s with the model OFF; the ≤0.45 figure is again accurate
  *because* the model is OFF).
- Deployment: `colcon build --packages-select jugglebot` + relaunch (the
  generated `hardware_config.py` lives inside the package).

## Verification

- Scoped (run 2026-07-17): `pytest tests/motion/test_retime.py
  tests/ros/test_trajectory_node.py tests/motion/test_shaped_batch.py -q` →
  **162 passed in 218.25 s** (all retime tests pass the flag explicitly, so
  the default flip changes no test behaviour).
- Full suite (`pytest tests/ -q`, run 2026-07-17, final pre-commit gate):
  **2873 passed, 1 xfailed in 766.44 s** (+1 = the new
  `test_shipped_retime_model_is_off_until_tracking_envelope_closed` tripwire).
- Hardware re-validation (operator): one bare battery run after
  `colcon build --packages-select jugglebot` + relaunch — expect yesterday's
  lean-0.6 feel back (traverse planned peaks back to ~122 mm/s / jerk ~12k,
  deviation ≤ ~0.45 rev + the day's sticky offset), planning pauses ~0.23 s
  (imperceptibly longer than this session's ~0.09 s).
- Forensics reproducible from
  `/home/jetson/Desktop/rosbags/2026-07-17_19-32-03` (extraction: scratchpad
  `ring_extract.py`, same pattern as the morning's `wobble_extract.py`).

## Addendum — 2026-07-18: the tracking envelope this entry cites is uptime-dependent

The ~0.6 ABORT-line breach that motivated shipping the model OFF was measured
at ~26 h of can-bridge Teensy uptime; tracking lag is now shown to grow
monotonically with Teensy uptime (fresh boot ≈10 ms). If the reboot
experiment confirms, the model's honest durations are likely trackable on a
healthy plant and the OFF decision should be revisited (it remains correct
*for a degraded plant*, and the honest-timing mechanism this entry documents
stands regardless). Also corrected by the 07-18 bisect: "restores the
hardware-validated behaviour byte-for-byte" holds for the timing *algorithm*,
not the lattice — the Phase-1b mesh commit shifted the legacy loop's
bisection lattice, so the post-OFF traverse plans at jpk ~20928 (not the
validated 07-16 sweep's 12000), the ~11 % real code term; this entry's
Verification expectation ("peaks back to ~122 mm/s / jerk ~12k, deviation
≤ ~0.45 rev") is therefore NOT exactly recovered by the flag alone. See
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`.

## Withdrawn claims

- *Operator hypothesis 1 — compute limit/bottleneck*: killed; emit gaps
  28.8 ms max, same as validated sessions.
- *Operator hypothesis 2 — SPE introduced ringing/instability*: killed; no
  oscillation signature post-arrival either day, and the non-traverse plans
  are near-identical. The rewrites changed *durations* on high-lean moves
  only, through the designed retiming path — not stability.
- *This morning's implicit claim (wobble entry) that the shipped lean-0.6
  configuration had comfortable margin (traverse ≤ ~0.45 rev)*: true only
  under legacy timing. It silently assumed the timing path that the same
  day's SPE deployment had replaced. Corrected by addendum there; true again
  now that the model ships OFF.

## Open Questions

- Re-enable criteria for the model: after accel FF lands, re-run this exact
  A/B (`19-32-03` recipe) and require the model-timed traverse deviation ≤
  the legacy-timed value + day noise. Alternatively, design the
  tracking-aware duration floor (plan-time bound on commanded leg velocity vs
  the measured chase envelope ~2.2 rev/s sustained) — that would close the
  whole gate-feasible-but-untrackable class for unshaped moves too.
- Add the ±150 traverse family to the retime optimality corpus
  (`tests/motion/test_retime.py`) so offline numbers bound the real battery's
  worst case.

## Related

- `logbook/2026-07-17-wobble-latch-unshaped-traverse.md` — the morning's
  unshaped cliff; same underlying class (gate-feasible but untrackable).
- `logbook/2026-07-17-shaped-planning-efficiency-implemented.md` — the SPE
  work; its Phase 2 correctness claims stand, its hardware outcome is this
  entry (addendum there).
- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the tracking-lag
  mechanism that makes honest durations untrackable.
- `plans/parked/accel-ff-inertia.md` — the fix that re-enables the model.
