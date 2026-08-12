---
title: Post-throw dip recurs at 1.0 m — the hand drive under-delivers braking current, not a Phase 1–4 regression
type: investigation
date: 2026-08-10
status: in-progress
phase: "catch-robustness Phase 0 (diagnosis)"
related_plan: catch-robustness.md
sessions:
  - 2026-08-10_16-04-26 (rosbag)
  - 2026-08-10_16-13-48 (rosbag)
  - 2026-08-10_16-30-44 (rosbag)
files_changed:
  - plans/active/catch-robustness.md
  - plans/active/hand-command-continuity.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
tags:
  - safety
  - dynamics
  - testing
---

# Post-throw dip recurs at 1.0 m — the hand drive under-delivers braking current

## Summary

The operator-visible post-throw dip came back on every 1.0 m throw of the
2026-08-10 tier-8a retest, alongside end-stop bumps and a ~60 % catch rate.
Offline analysis of the three retest bags says this is **not** the 2026-07-25
mid-stroke-clobber class returning and **not** a missing FW-2 feedforward: the
arm gate holds on 54/54 strokes and the firmware commands its full decel
feedforward every time. What the plant does not do is *deliver* it — the hand
ODrive's braking current never exceeds ≈ −11.8 A anywhere in 204 k samples while
the same channel resolves +49.6 A on the motoring side. The commanded
feedforward asks for **−27.2 A**. The shortfall is one-sided, which points at a
drive-side negative-torque/current limit rather than at physics, and it lands
squarely on a risk this repo pre-registered a fortnight ago: C-HAND-2's
`axis0.config.torque_soft_min = −0.055133 N·m = exactly −10.00 A`. No fix is
landed — this entry is the diagnosis and the bench pre-flight it hands to the
operator.

## Symptoms

Owner-run tier-8a retest, 2026-08-10, three bags at ~16 h can-bridge uptime
(tilt map live, `catch_vel_scale_default` 0.9):

- a returned post-throw **dip** on every 1.0 m throw — the hand drops below the
  top of its stroke before the catch descent, which the operator sees as a
  visible bob;
- light **end-stop bumps** at the top of the 1.0 m stroke;
- catch rate at 1.0 m around **60 %**, well short of the programme goal.

The dip is the same *symptom* that `plans/active/hand-command-continuity.md`
Phases 1–4 were built to remove, so the first question was whether that fix had
regressed.

## Diagnosis

### 1. The Phase-1 arm gate holds — the clobber class is NOT back

`tools/probes/hand_stroke_timeline.py` over the three bags: 81 announcements,
57 jugglebot strokes, of which **54 at v_cmd 4.436 m/s** (the 1.0 m tier) and 3
at 3.440 m/s.

| at v_cmd 4.436 m/s, n = 54 | min | median | max |
|---|---|---|---|
| `dip_below_x3` (band 0.10 rev) | 0.268 | 0.899 | 1.672 rev |
| `peak` (stroke limit 11.10 rev) | 10.468 | 10.700 | 10.766 rev |
| coast past `x3` | 0.508 | 0.741 | 0.807 rev |
| `pullback` | −12.9 | −16.75 | −19.3 rev/s |
| `first_neg_cmd` | −7.4 | −11.65 | −15.6 rev/s |

**54/54 are OVER the 0.10 rev band** — but `trunc = -` and `seeds = 0` on all
54. The commanded position follows the planned decel ramp to `x3` on every
stroke and no from-rest quintic is seeded inside any stroke, which is exactly
the Phase-5 PASS shape for those two rows. The three strokes in the set that
*do* print a `trunc`/`seeds` row are all at 3.440 m/s, and each reports
`|cmd-meas| = 0.0000 rev` at the "freeze" with `dip_below_x3 = 0.000`; each
"freeze" lands 37.8 / 42.9 / 43.7 ms after the modelled stroke end, i.e. inside
`_TRUNC_SCAN_MARGIN_S = 0.050` — the known criterion defect recorded against the
2026-07-27 sitting (§ Implementation Phase Summary row 5), not a real truncation.

Peak headroom is the other half of the operator's report: the worst stroke
reaches 10.766 rev against the 11.10 rev declared limit, **0.334 rev ≈ 10.6 mm**
of margin, which is what the end-stop bumps are.

So the dip is being produced *while the arm gate is doing its job*.

### 2. FW 2 is aboard and commanding correctly

`tor_ff_cmd` reads **−0.1500 N·m on 54/54** of the 1.0 m strokes (and −0.0900
N·m on the 3.44 m/s tier). That is `throwDecelToTorque` at the declared
`throw_decel_reflected_inertia_kgm2 = 9.5e-6`. Checked rather than assumed:
`a_cmd = v/t_dec = (4.436 / 0.0316) / 0.0577 = 2433 rev/s²`, so
`τ = J·2π·a_cmd = 9.5e-6 × 2π × 2433 = 0.1452 N·m` → **−0.15** on the 0.01 N·m
wire quantum. The FW-1 `accelToTorque` spool model (7.3695e-6 kg·m², i.e. 77.6 %
of the declared value) gives 0.1127 N·m → **−0.11** on the wire. The two are not
confusable, so the flash landed and the feedforward is commanded at full size on
every stroke.

### 3. The drive does not deliver it, and the shortfall is one-sided

At the hand's `torque_constant` 0.0055133 N·m/A, −0.1500 N·m is a request for
**−27.2 A** of braking current. Measured, across all three bags:

| | min | median | max |
|---|---|---|---|
| `iq_meas` inside the decel-FF ramp, 1.0 m tier (n = 54) | −10.52 | −7.60 | −3.85 A |
| `iq_meas` motoring, 200 ms before the same ramps | +14.93 | +22.80 | +28.49 A |

and, per bag, the whole-session extremes on the same channel — the negative one
first, the positive one second:

| bag | most negative `iq_meas` | most positive `iq_meas` |
|---|---|---|
| 16-04-26 | −11.60 A | +42.19 A |
| 16-13-48 | −11.76 A | +28.49 A |
| 16-30-44 | −11.42 A | +49.64 A |

**The negative side has a hard floor at ≈ −11.8 A across 203 922 samples; the
positive side runs to +49.6 A.** The channel is not blind to large currents and
the plant is not weak — it is one-sided.

Freshness was checked rather than assumed, because `iq_meas` is a known-aliased
channel (C-HAND-2 § *Predicted effect*): median repeat-run is 2–3 samples on
these bags (≈ 33–50 Hz effective refresh), giving **median 4 FRESH samples
inside each decel ramp**, ≈ 216 fresh braking samples over the 54 strokes. The
decel feedforward is a plateau, not a spike, so a −27 A plateau could not hide
between four samples 54 times over.

### 4. The named candidate

`config/ODrive config Files/odrive_pro_hand_config.json` declares
`axis0.config.torque_soft_min = −0.055133331567049026 N·m`, which at this Kt is
**exactly −10.00 A**, against a `torque_soft_max` of +0.5 N·m (+90.7 A, so the
50 A current limit is what binds on the motoring side). The measured floor
matches the declared clamp to about 1 A, and the observed asymmetry matches the
declared asymmetry. That value is not new information — it is C-HAND-2
§ *The negative torque clamp*, bench pre-flight **H7.0c**, and the named
failure-table row in `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-7
("every rung reads its pre-fix peak, H7.0 says v2, and the flatness row also
fails ⇒ the negative torque clamp is live"). That is the state the plant is in.

### 5. How the dip is actually produced

With braking clamped, the stroke coasts **0.51–0.81 rev past `x3`** instead of
settling onto it. The position loop then reacts at −12.9…−19.3 rev/s. On top of
that, the gated hand-catch arm lands **+31.9 / +44.8 / +57.7 ms (min/med/max)
after the commanded stroke end**, while the hand is still moving at ≈ −11.7
rev/s — above `SMOOTH_MOVE_V0_DEADBAND_RPS = 6.0`, so the FW-2
velocity-continuous prelude fires with a large `v0`, and its bulge plus the
loop's undershoot carries the hand below `x3`. The dip is therefore a
*consequence* of the clamped braking, arriving through machinery that is
working as designed.

### 6. Two collateral findings

- **Dispatch shift has grown with uptime.** fit − announcement is
  +117.5…+132.9 ms (per-bag medians +124.9 / +122.2 / +123.8), against the
  `+54…+63 ms` recorded on 2026-07-27 (itself already over the 40 ms margin
  Phase 1's stroke-busy window budgets) and `+12.8…+21.9 ms` pre-fix. Consistent
  with the standing bridge-uptime-lag item; these bags are ~16 h uptime.
  Fresh-boot-before-every-sitting discipline stands, and `uptime_ms` must be
  logged with any timing number.
- **`iq_meas` was frozen for the first 171.71 s of bag 16-04-26** — one
  contiguous run of 17 138 samples at exactly −10.13 A, at rest and in motion
  alike, after which the channel updates normally. The earliest stroke in that
  bag starts at t = 178.00 s, so no stroke measurement is contaminated, but any
  future analysis of that bag must exclude the window. (Do not mistake the
  −10.13 A artefact for the −10.00 A clamp: it is a stale value, not a current.)

### 7. The sensor ledger is clean, and it already carries catch labels

`ball_held_valid` is true on **100 % of samples in all three bags** (47 124 /
86 132 / 70 666). Debounced `ball_held` transitions, over valid samples, on bag
log_time:

| bag | announcements | held→empty | empty→held | quick-drops (<1.5 s) |
|---|---|---|---|---|
| 16-04-26 | 14 | 6 | 5 | 0 |
| 16-13-48 | 27 | 23 | 24 | 0 |
| 16-30-44 | 40 | 39 | 38 | 3 |

Announcements include BallButler reload throws, so the count is not a toss
count. The three quick-drops in 16-30-44 are bounce-outs — a catch label the
sensor produced with no wiring in place, mined offline. That is the evidence
that made the owner's Phase-1 decision (sensor becomes the possession source of
truth and the ball-evidence gate) cheap to make.

## Discussion

### The hypothesis that was withdrawn, and what killed it

The session opened on **"the arm is clobbering the stroke again, and bridge
uptime is why"**. It was a good hypothesis: the symptom is the same one Phases
1–4 fixed, the plant had ~16 h of uptime, dispatch shift *had* grown from
+54–63 ms to +118–133 ms, and a late-enough arm dispatch is precisely what the
Phase-1 gate exists to prevent. If the gate's stroke-busy window were being
outrun by a grown dispatch latency, the arm would land mid-stroke, clear the
packed queue and re-prelude from the live encoder — the 2026-07-25 class,
verbatim.

The probe refutes it in one row: a mid-stroke clobber **must** print a `trunc`
(commanded position freezing at the live encoder value) and at least one
from-rest `seeds` entry. Across 54 strokes at the tier that dips, there are
**zero** of each. Whatever produces the dip, it is not the queue being cleared.
The gate holds. The uptime story survives only as a separate, real, but
*non-causal* observation about dispatch latency.

Rescuing the hypothesis was available and was rejected: one could argue the
clobber happens in a way the probe's criteria miss. That argument requires the
criteria to fail on 54 consecutive strokes while succeeding on the synthetic
two-sided gate the probe ships with — an appeal that costs more than it buys.
The evidence killed the hypothesis, so the hypothesis goes.

### Why the feedforward-vs-current asymmetry pins the drive side

Three explanations can produce "the hand does not brake hard enough":

1. **The firmware is not commanding the braking** (FW 1 still aboard, or the
   feedforward is being computed small). Excluded: `tor_ff_cmd` reads −0.1500
   N·m on 54/54, which is the FW-2 value and is 29 % larger than the FW-1 model
   would produce (0.1452 vs 0.1127 N·m before quantisation).
2. **The plant is heavier / draggier than modelled**, so the commanded braking
   is simply not enough. This is the *physics* explanation, and it is the one
   Phase 7 was written for. It predicts a **two-sided** signature: a plant that
   needs more braking torque than modelled also needs more motoring torque than
   modelled, and both sides of the stroke draw hard.
3. **The drive refuses to source the current in the braking direction.** This
   predicts a **one-sided** signature with a *hard floor*: the negative extreme
   pins at a value and stops, while the positive extreme is free.

The measurement is unambiguously (3). Over 203 922 samples the negative extreme
is −11.8 A and the positive extreme is +49.6 A — a factor of 4.2 apart on the
same channel in the same session. And the kinematics say the two sides should be
**nearly equal**: at this tier the stroke accelerates over `t_acc = 77.2 ms` and
decelerates over `t_dec = 57.7 ms` to the same Δv ≈ 4.42 m/s, so
`a_acc ≈ 57.3 m/s²` and `a_dec ≈ 76.6 m/s²`; the hand is travelling *upward*
through both, so gravity opposes the accel and assists the decel, and
`(a_dec − g) / (a_acc + g) = 66.8 / 67.1 = 0.995`. Motoring measures +22.8 A
median. Braking should measure about the same and measures −7.6 A.

Friction is the one alternative that can bend a one-sided story, since it
opposes motion in both phases. Making it fit requires a friction term worth
≈ 3.4 g — about half the total accel force — which would then demand a motoring
current far above the +22.8 A actually observed. It does not survive its own
arithmetic.

### Why this is a *recurrence* of a pre-registered risk, not a new discovery

The Phase-7 adversarial review already flagged
`axis0.config.torque_soft_min = −0.0551 N·m = −10.00 A` as the thing that could
make the whole feedforward phase a no-op, and it wrote the pre-flight (**H7.0c**,
30 s with `odrivetool`) plus the in-band discriminator (H7.3 flatness) rather
than treating it as proven. It marked the clamp *probably not binding* on
counter-evidence: a hard clamp would force one single achieved deceleration at
every tier, and the 2026-07-27 capture's achieved decel grew 2.6× across the
band.

That counter-evidence is about a **different physical drive**. The hand ODrive
was swapped during the 2026-07-31 CAN3 elimination, and the owner reports this
machine's ODrives occasionally come up needing config resets (a reset also cured
the 2026-08-10 leg-0 spinout). A configuration flag can be dormant on one board
and live on its replacement; the 2026-07-27 argument does not transfer, and the
2026-08-10 data is the first look at the new drive under a real decel ramp.

The honest statement of what is proven: **the braking authority is capped, the
cap is drive-side, and the declared `torque_soft_min` matches the cap to about
1 A.** What is not proven is that `torque_soft_min` is the *live* register — a
backup JSON is not the axis. That is what H7.0c reads, and it is why Phase 0 is
a config dump-and-diff rather than a config write.

### The tension the bench has to resolve, recorded rather than smoothed over

A clamp binding at exactly −10.00 A does not cleanly reproduce every number
here. The 3.44 m/s tier tracks its commanded decel almost perfectly
(`over_x3` 0.058–0.070 rev, so `eta ≈ 0.98`), which at the C-HAND-2 declared
lower bound `J ≥ 1.0126e-5 kg·m²` needs ≈ 0.091 N·m of braking torque — more
than a −0.0551 N·m clamp plus gravity can supply. Two ways out, and they have
different consequences:

- the clamp truncates the **feedforward term only** and the position-loop output
  is added after it, in which case the loop supplies the rest and the clamp is
  still the thing to fix; or
- the clamp truncates the **total**, in which case the true reflected inertia is
  much smaller than 1.0126e-5 — because that "decel-side lower bound" was
  derived from achieved decel *assuming the commanded feedforward was
  delivered*, and it was measured through the clamp. Ball-corrected accel-side
  identification gave 7.74e-6 and the naive spool model 7.3695e-6; a
  clamp-corrected re-derivation lands near those.

The second branch would mean Phase 7's inertia re-declaration has been
compensating for a drive-config clamp rather than for inertia, and C-HAND-2's
declared bound would have to be re-derived on a restored drive. **Nothing is
changed on that basis today.** The measurement that discriminates is the one the
bench is about to make: restore the drive, re-run the R0→R5 ladder, and see
whether achieved decel becomes tier-independent.

### Why no catch tuning happens before the bench

The catch seat decelerates the hand in the **same regen direction** as the throw
brake. If the drive is clamped, catch-knob values tuned against today's plant
are fitted to a defect, and every one of them would have to be re-tuned after
restoration. The 60 % catch rate is therefore a measurement of a degraded plant,
not a baseline, and `plans/active/catch-robustness.md` carries it as a hard
constraint on Phases 1–3.

## Fix

**None landed.** This entry is diagnosis and hand-off; the corrective action is
a bench sitting the operator runs.

Documentation changes only:

- `plans/active/catch-robustness.md` (new) — the programme plan, with Phase 0
  naming the config read and the restoration gate;
- `plans/active/hand-command-continuity.md` — a dated addendum at Phase 5
  recording that the dip recurred and why it is not a Phase 1–4 regression;
- `plans/active/INDEX.md` — the new plan's row (pinned by
  `tests/sim/test_plans_index.py`, which is why it lands in this commit);
- `logbook/INDEX.md` — this entry's row.

## Verification

Every number above was re-derived this session against the bags, not copied
forward. All runs 2026-08-10, project venv (`source ~/Desktop/PDJ_venv/venv/bin/activate`):

1. **Stroke shape, arm gate, dispatch shift** —
   `python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/2026-08-10_16-04-26 --bag ~/Desktop/rosbags/2026-08-10_16-13-48 --bag ~/Desktop/rosbags/2026-08-10_16-30-44`
   → 81 announcements, 57 jugglebot strokes; at 4.436 m/s (n = 54)
   `dip_below_x3` 0.268/0.899/1.672 rev with **54/54 OVER** the 0.10 rev band,
   `peak` 10.468/10.700/10.766 rev, **0 truncations and 0 from-rest seeds**;
   per-bag dispatch-shift medians +124.9 / +122.2 / +123.8 ms.

2. **Commanded torque, braking current, sensor ledger** — the recipe below
   (offline, read-only; `mcap_ros2` decodes without ROS2):

   ```bash
   source ~/Desktop/PDJ_venv/venv/bin/activate
   python - <<'PY'
   import glob, statistics as st
   from mcap_ros2.reader import read_ros2_messages
   BAGS = ['2026-08-10_16-04-26', '2026-08-10_16-13-48', '2026-08-10_16-30-44']
   brake, motor = [], []
   for b in BAGS:
       r = []
       for f in sorted(glob.glob(f'/home/jetson/Desktop/rosbags/{b}/*.mcap')):
           for m in read_ros2_messages(f, topics=['/hand_telemetry']):
               g = m.ros_msg
               r.append((m.log_time_ns/1e9, g.tor_ff_cmd, g.iq_meas,
                         g.ball_held, g.ball_held_valid))
       r.sort()
       eps, cur = [], []
       for s in r:                    # a decel-FF episode = tor_ff_cmd < -0.05 N.m
           if s[1] < -0.05: cur.append(s)
           elif cur: eps.append(cur); cur = []
       for ep in (e for e in eps if min(x[1] for x in e) == -0.15):
           brake.append(min(x[2] for x in ep))
           motor.append(max((x[2] for x in r if ep[0][0]-0.2 <= x[0] < ep[0][0]), default=0))
       h2e = e2h = quick = 0; prev = None; t_h = None
       for s in (x for x in r if x[4]):
           if prev is None: prev = bool(s[3]); t_h = s[0] if s[3] else None; continue
           if bool(s[3]) != prev:
               if s[3]: e2h += 1; t_h = s[0]
               else:
                   h2e += 1
                   quick += (t_h is not None and s[0]-t_h < 1.5)
               prev = bool(s[3])
       print(f'{b}: {len(r)} samples  iq {min(x[2] for x in r):+.2f}..{max(x[2] for x in r):+.2f} A  '
             f'valid {sum(x[4] for x in r)}/{len(r)}  held->empty {h2e}  empty->held {e2h}  quick {quick}')
   f = lambda v: f'{min(v):+.2f}/{st.median(v):+.2f}/{max(v):+.2f}'
   print(f'1.0 m tier (tor_ff_cmd = -0.1500 N.m) n={len(brake)}  '
         f'iq_brake {f(brake)} A   iq_motor {f(motor)} A')
   PY
   ```

   → ```
   2026-08-10_16-04-26: 47124 samples  iq -11.60..+42.19 A  valid 47124/47124  held->empty 6  empty->held 5  quick 0
   2026-08-10_16-13-48: 86132 samples  iq -11.76..+28.49 A  valid 86132/86132  held->empty 23  empty->held 24  quick 0
   2026-08-10_16-30-44: 70666 samples  iq -11.42..+49.64 A  valid 70666/70666  held->empty 39  empty->held 38  quick 3
   1.0 m tier (tor_ff_cmd = -0.1500 N.m) n=54  iq_brake -10.52/-7.60/-3.85 A   iq_motor +14.93/+22.80/+28.49 A
   ```

   The `tor_ff_cmd == -0.15` filter selecting exactly **54** episodes — the same
   54 the stroke probe counts at 4.436 m/s — is itself the cross-check that the
   FW-2 feedforward fired on every 1.0 m stroke.

3. **The declared clamp, and the 2026-07-27 pre-swap baseline** —
   `python tools/probes/hand_decel_authority.py --trace temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`
   → prints `!! hand ODrive axis0.config.torque_soft_min = -0.055133 Nm = -10.00 A
   (asymmetric; torque_soft_max = +0.5 Nm)` and the pre-swap per-tier table whose
   achieved decel grows 911 → 2330 rev/s² across the band (the counter-evidence
   discussed above, and the reason it does not transfer to the swapped drive).

4. **Test gate** — `./run_tests.sh`, run 2026-08-10: **4275 passed, 0 failed in
   189.34 s** (parallel phase; the gate's serial phase is empty because every
   `serial`-marked test is also `nightly`), total 201 s, `RESULT: PASS`.
   The surface this commit touches is inside the suite: `tests/sim/test_plans_index.py`
   pins every `plans/active/*.md` against its `INDEX.md` row (so the new plan and
   its row must land together), and `tests/sim/test_logbook_front_matter.py` +
   `tests/sim/test_logbook_search.py` parse the real `logbook/` tree, failing on a
   missing required key, a non-ISO `date`, or anything that makes the loader warn.
   What they do **not** check — per `logbook/README.md` § *What the logbook tests
   actually check* — is `logbook/INDEX.md` (skipped outright) or the body of a new
   entry, so the INDEX row and the prose above were checked by hand, not by the
   passing count.

## Outcome

The dip is characterised, the Phase 1–4 fix is exonerated, and the next action
is a 30-second read on the live hand axis rather than another tuning sitting.
`plans/active/catch-robustness.md` Phase 0 carries it: dump and diff the hand
ODrive config against `config/ODrive config Files/odrive_pro_hand_config.json`
(`torque_soft_min` first, then the regen/`dc_max_negative_current` family and
`torque_constant`), restore, fresh can-bridge boot, then the HAND-7 R0→R5 ladder
and a 1.0 m retest gated on `dip_below_x3 ≤ 0.10 rev` and `peak ≤ 10.060 rev`.

Test gate for the documentation commit: `./run_tests.sh`, run 2026-08-10 —
**4275 passed, 0 failed in 189.34 s**, `RESULT: PASS`, exit 0.

## Withdrawn claims

- [2026-08-10] Claimed the recurring dip was the **2026-07-25 mid-stroke
  clobber class returning**, driven by a hand-catch arm dispatch that grown
  bridge-uptime latency had pushed inside the throw stroke.
  WITHDRAWN: across 54 strokes at the tier that dips, the stroke timeline shows
  `trunc = -` and `seeds = 0` on every one — a queue-clearing re-prelude cannot
  occur without freezing `pos_cmd` at the live encoder value and seeding a
  from-rest quintic, and neither appears. The Phase-1 arm gate holds.
  Superseded by: § Diagnosis 1 and 3 — the braking current is clamped
  drive-side. The dispatch-shift growth (+118…+133 ms) is real and recorded, but
  it is not the cause of the dip.

## Open Questions

1. **Which register is actually live?** `torque_soft_min` matches the observed
   floor to ~1 A, but a backup JSON is not the axis. H7.0c reads it; the same
   dump should capture the regen family (`dc_max_negative_current`,
   `dc_bus_overvoltage_ramp_*`) and `torque_constant`, since any of them can cap
   braking one-sidedly and the drive was swapped 2026-07-31.
2. **Does C-HAND-2's declared inertia survive restoration?** Its decel-side
   lower bound `J ≥ 1.0126e-5 kg·m²` was measured *through* whatever cap was in
   force. If the cap was binding, the bound is inflated and
   `throw_decel_reflected_inertia_kgm2` should be re-derived on a restored
   drive, not before. Discriminator: whether achieved decel becomes
   tier-independent after restoration.
3. **Does the same clamp degrade the catch seat?** The catch decelerates in the
   same regen direction. Untested — and the reason no catch knob is tuned before
   Phase 0.
4. **Why did `iq_meas` freeze for 171 s at the head of bag 16-04-26?** Frozen at
   a single value at rest and in motion, then healthy. Not investigated; it did
   not touch any stroke measurement here, but it is a telemetry-integrity
   question for the hand's CAN read path.
5. **Bridge-uptime dispatch lag** (+54–63 ms fresh → +118–133 ms at ~16 h) —
   unchanged in status, owned by `plans/active/refactor-2026-07.md` Phase 7.
