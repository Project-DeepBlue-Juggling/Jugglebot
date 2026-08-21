---
title: Two MAX_DEVIATION latches on the first post-SPE session were the known unshaped-traverse cliff, not an SPE regression — plans byte-identical to yesterday's 0.94-rev run; the battery's lean default silently disabled the shipped lean
type: investigation
date: 2026-07-17
status: resolved
phase: "MVP trajectory bringup — S4 working point on hardware (first session after SPE + persistence)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - tests/hardware/traj_ramp_battery.py
  - tests/hardware/mvp_bench_runbook.md
  - logbook/2026-07-17-s4-closed-working-point-persisted.md
commits:
  - 2ac20ae
subsystem:
  - motion
  - ros
tags:
  - max-deviation
  - guard
  - tracking-lag
  - lean-shaping
  - traj-ramp-battery
  - working-point
---

## Summary

The first hardware session after the SPE deployment + persisted working point
(bag `2026-07-17_17-35-14`) hit two MAX_DEVIATION E-STOPs within a few moves,
with the platform "wobbling fairly violently", on both a bare battery run and a
`--lean-gain 0.0` run. Forensics on the bag exonerate every code change shipped
since the last validated session: **the planner's output is numerically
identical to 2026-07-16's validated runs** (matched moves agree in planned peak
vel/acc/jerk to the decimal — 182.5/1178.1/25915 etc.), the emit cadence is
clean (25–27 ms), and the retiming model never engages (both runs were
unshaped). The latch is the **pre-existing unshaped ±150 lateral-traverse
cliff** already visible in yesterday's own record at 0.94 rev (94 % of guard):
today the same move rode 1.02–1.08 rev on an ordinarily stickier plant
(first-moves-cold; +10–30 % lag at matched commands with identical peak chase
velocity and higher iq) and crossed the 1.0 rev latch — twice, both times on
the x +150→−150 traverse, leg 1 deepest (consistent with 2026-07-16).

Both runs were unshaped because `traj_ramp_battery.py`'s `--lean-gain` default
was `0.0` — under the b6391c1 convention that is the EXPLICIT lean-off arm, so
the operator's "defaults" run and "--lean-gain 0.0" run were identical configs
and the shipped lean 0.6 never ran. Under lean 0.6 the same traverse plans at
~122 mm/s leg peak and rides ≤ ~0.45 rev (2026-07-16 sweep). Fix: the battery
default is now `-1.0` (defer to shipped config), and the record now states
plainly that **unshaped ±150 traverses at jerk 30000 are out of the trackable
envelope until accel FF lands** — a latch there is expected behaviour, not a
regression.

## Symptoms

- Operator: "with both defaults and --lean-gain 0.0, the platform wobbled
  fairly violently, exceeding MAX_DEVIATION in just a few moves."
- Session `2026-07-17-17-35-13` (launch log) + bag `2026-07-17_17-35-14`:
  TRAJECTORY entered t=17.8 s and t=50.1 s; `ACTIVE → FAULT (forced)` at
  t=33.6 s (move 4) and t=62.1 s (move 10; move numbers are the session-global
  `move_seq` from `/trajectory/diagnostics`).
- Guard latch snapshots (`/link_status`): both trips `max_dev_leg=1`,
  `max_dev_value=−1.03 / −1.00` (u0 1.61/1.77 vs enc 2.63/2.78 — the command
  dived faster than the plant could follow; the encoder was *above* the
  falling command, not runaway).

## Diagnosis

1. **Deployment state verified first**: installed `retime.py`/`planner.py`/
   `feasibility.py`/`shaping.py` match the SPE commits (colcon preserves source
   mtimes — the "stale-looking" timestamps are the files' edit times);
   installed `GoToPose` srv carries `lean_gain` default `-1.0`. The full SPE
   stack + b6391c1 were live. Not a stale-install problem.
2. **Both runs were lean 0.00** (`/trajectory/diagnostics` `lean_gain=0.00` on
   every move, and per-move planned peaks identical across the two runs).
   `traj_ramp_battery.py` line 70: `--lean-gain` default `0.0`, always sent →
   explicit lean-off. The operator's intended A/B compared two identical arms.
3. **The plans are byte-identical to the validated 2026-07-16 session.**
   Matched moves, bag `21-58-59` (vlim 2000 arm) vs today (vlim 1000 — these
   battery moves are jerk/acc-bound, so the vel limit is inert):

   | move | planned peak vel/acc/jerk (both days) | dev 07-16 | dev 07-17 |
   |---|---|---|---|
   | z 170→220 | 182.5–182.9 / 1178–1179 / 25915 | 0.344 | 0.448 |
   | x +150 | 224.8 / 1335.5 / 25915 | 0.493 | 0.629 |
   | x +150→−150 | **312.5 / 1621.2 / 25915** | **0.936** | **1.082 → LATCH** |
   | (run 2, same traverse) | 312.5 / 1621.2 / 25915 | — | 1.022 → LATCH |

   Identical planned peaks to the decimal ⇒ the SPE commits (4acaefe, 54c1e75,
   806e8fb) and b6391c1 changed nothing about these commands. The retiming
   model is shaped-only and never engaged; the batched gate is shaped-only;
   emit gaps 25–27 ms all session. Note also the x+150 *single* rode 0.629
   today — already above the recalibrated ~0.6 rev in-move ABORT line — i.e.
   the unshaped battery at this working point brushes the operator ABORT
   criterion even on singles.
   Reconciliation with the 2026-07-16 lean entry's "lean 0.6 and 1.0 →
   0.25 rev": that figure was not the traverse — re-extraction of the sweep
   bag (`22-06-30`, same vlim 2000 / jerk 30000) puts the lean-0.6 traverse at
   **0.450 rev** (lean 1.0: 0.460; arm-worst is the tilt move at 0.568, which
   lean does not shape). Like-for-like on the traverse the lean benefit is
   0.936 → 0.450, not 0.94 → 0.25.
4. **The mechanism is 2026-07-16's velocity-loop lag, unchanged.** The
   traverse commands 4.43 rev/s leg peaks; the plant chases at ~2.0–2.7 rev/s
   under coordinated load (measured both days, matched). The deficit
   integrates over the 1.55 rev stroke → ~1 rev deviation. iq peaks 6–8 A,
   `iqm≈iqs` — drives never railed; the loop simply doesn't ask for more
   (vel_gain 0.20 softness under reflected load — the accel-FF chapter's
   target).
5. **The day-to-day delta (+10–30 % at matched commands) is plant-side, not
   code**: matched z-moves reach identical peak encoder velocity (~1.9–2.0
   rev/s) with *higher* current today (5.4 vs 4.6 A) — a stickier/colder day.
   (Per-move deltas: z +30 %, x+150 single +28 %, traverse +16 %/run 1 and
   +9 %/run 2.)
   Today's trips were the first passes through the hot move each run
   (yesterday's matched pass came 44 moves into a warm session); run 2 was
   already 6 % better than run 1 (1.022 vs 1.082). Bus voltage is not in the
   bag, so the exact physical driver (SOC vs temperature) is unproven.
6. **"Violent wobble"** = the chase dynamics of back-to-back short moves (the
   command completes in 0.5–0.7 s; the plant arrives ~0.5 s later at 2–2.7
   rev/s and 5–8 A — a lurching catch-up on every move) plus the E-STOP
   cutting the 300 mm/s traverse mid-flight. The battery's pacing itself is
   correct (`max(settle, planned+0.5 s)`, no overlapping installs).

## Discussion

**Why SPE was the prime suspect, and what actually exonerated it.** A 12×
planning speedup shipped hours before a violent latch is an obvious causal
candidate, and the failure signature (encoder "ahead" of command) initially
looked like a command-side discontinuity — an install-continuity or retiming
bug. Two things killed that line cleanly: the negative deviation is just the
command *diving* faster than the plant falls (sign convention, not overshoot),
and — decisively — the matched-move table: identical planned peaks to the
decimal across four move types on both days. A duration/gate regression cannot
produce byte-identical plans. This is the cheap, high-power test for "did the
planner change?": diff the per-move `peak_leg_*` diagnostics across sessions
before reading a line of code.

**The process failure worth recording: a 0.064 rev margin was treated as a
positive.** The S4-closure commit (b6391c1) chose 30000 jerk over 40000
*because* its guard margin was better — 0.064 vs 0.027 rev. Both numbers are
knife-edge: 6 % and 3 % of a safety latch, against day-to-day plant variation
that today measured 10–30 %. The caveat WAS recorded (runbook S4 Result block:
"94 % of the guard") but framed as a footnote to a PASS rather than as what it
is — the unshaped hot lateral is **out of envelope**, and any session that runs
it unshaped is sampling a distribution whose upper tail crosses the latch. The
correct statement all along was: the working point is validated *for the
shipped configuration (lean 0.6)*; its gain-0 arm is a stress case that fails
on ordinary bad days. Margin on a latch must be compared against observed
run-to-run variance, not against zero.

**Why the fix is the instrument default, not a limits retune.** No explored
session-limit value gives the unshaped ±150 traverse a healthy margin: even
the gentle jerk-8638 rung rode 0.73 rev (2026-07-16, seq 4), because the
deviation is dominated by stroke × velocity-deficit fraction, and the deficit
exists at any jerk that makes the move look "dynamic". Lowering the persisted
jerk to where the unshaped traverse is comfortable (~0.5 rev) would roughly
halve every move's character — paying permanently, in the shipped config, for
a configuration (lean OFF) that ships disabled. Meanwhile the shipped config
already has the margin: lean 0.6 stretches exactly the high-lateral-accel
moves (traverse ≤ ~0.45 rev, worst battery move ~0.57 at tilt). So the correct
scope is: (a) make the battery actually run the shipped config by default —
the `-1.0` defer convention its own help text already described — and (b)
state the unshaped-traverse cliff plainly in the runbook and this entry so the
next latch there is read as expected, not as a regression. The real margin
restoration for unshaped aggressive moves remains the accel-FF chapter
(`plans/parked/accel-ff-inertia.md`) — feedforward, not feedback, closes a
velocity-deficit this size (same conclusion as the 2026-07-13 gain-hunt
closure).

**The srv-default lesson bit again, one layer up.** b6391c1 fixed the
node-side reachability of the config lean (`-1.0` field default) and even
noted "the battery instrument's explicit `--lean-gain 0.0` default keeps its
baselines unshaped" — treating that as a feature. What nobody asked was
whether the *operator's mental model* of a bare run ("defaults") matched the
instrument's (explicit OFF). It didn't, and the mismatch cost a diagnosis
session: the A/B the operator believed they ran (shipped vs lean-off) was
identical arms. When a convention changes (unset = defer), every client
script's default is part of the convention's blast radius.

## Fix

- `tests/hardware/traj_ramp_battery.py`: `--lean-gain` default `0.0 → -1.0`
  (defer to `trajectory_op.lean_gain`); help text states `0.0` is the explicit
  unshaped baseline arm and names it the guard-latch-prone arm on hot
  laterals. Docstring: stale battery geometry corrected (z 220, x/y ±150,
  tilt ±10°), the pre-recalibration ABORT criterion ("tracking > 0.1 rev")
  replaced with the 2026-07-16 recalibrated rules, and the unshaped-traverse
  cliff documented with measured numbers.
- `tests/hardware/mvp_bench_runbook.md`: S4 Result block upgraded from
  "caveat" to confirmed cliff (today's 1.02–1.08 rev data, both latches, plan
  identity, expected-latch framing until accel FF); S2 battery bullet updated
  for the new default (explicit `--lean-gain 0.0` now required for S2's
  original conditions) and the widened geometry.
- `logbook/2026-07-17-s4-closed-working-point-persisted.md`: addendum
  correcting the margin framing (see that entry).
- No planner/gate/firmware/config change: the control path is behaving as
  designed and as previously measured.

## Verification

- Forensics reproducible from `/home/jetson/Desktop/rosbags/2026-07-17_17-35-14`
  (extraction script pattern: scratchpad `wobble_extract.py`, a copy of the
  2026-07-16 `sweep_extract.py` with the bag path swapped).
- Suite (`pytest tests/ -q`, run 2026-07-17, final pre-commit gate): **2872
  passed, 1 xfailed in 766.28 s** (an identical first gate ran after the code
  edit: 2872/1 in 770.07 s).
- Hardware re-validation (operator, next session): one bare battery run —
  expect shaped moves (planned durations visibly longer on x/y, per-move
  `lean_gain 0.60` in `/trajectory/diagnostics`), zero latches; optionally one
  `--lean-gain 0.0` run *stopping before the x-traverse* if an unshaped
  baseline is still wanted.

## Addendum 2 — 2026-07-18: the envelope numbers themselves are now suspect

*(Placed first as the operative caveat; the same-evening addendum below
predates it.)* The "out of the trackable envelope" framing (and the ±10–30 %
"sticky-day" attribution) was measured at 24–30 h of can-bridge Teensy
uptime; tracking lag is now shown to grow monotonically with Teensy uptime
and the fresh-boot plant tracked 3.5 rev/s commands at 0.01–0.06 rev
deviation. If the reboot experiment confirms, the unshaped traverse is likely
trackable on a healthy plant and the latches in this entry were the uptime
bug crossing the guard. The 07-18 sweep also disproves this entry's "bus
voltage is not in the bag" claim (Diagnosis §5 / Open Questions): it is, at
`motor_states[].bus_voltage`, and it was flat 45.0–45.3 V across all seven
sessions — the SOC candidate is dead. And a correction to the addendum below:
its "≤ ~0.45 is accurate again" assumed the flag flip restored the 07-16
lattice — the 07-18 bisect shows the mesh commit shifted the legacy traverse
to jpk ~20928 (was 12000), so the post-OFF traverse is hotter than the
validated sweep. See
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`.

## Addendum — same evening: the ≤ ~0.45 rev lean-margin figure assumed the legacy timing path

The operator's follow-up A/B session (bag `19-32-03`) found the shipped
lean-0.6 configuration riding **0.727 rev** on the x-traverse — not the
≤ ~0.45 this entry promised. This entry's margin figure was measured from the
2026-07-16 sweep, which was timed by the **legacy** stretch+bisection loop;
the same day's SPE deployment had switched shaped timing to the retiming
model, which removes the legacy overshoot that was largest on exactly the
lean traverses (peak jerk 12000 → 27406). No ringing, no compute stall — the
model's honest durations exceed the tracking envelope. `retime_model` now
ships **false** (legacy timing restored; ≤ ~0.45 is accurate again); full
analysis in `logbook/2026-07-17-retime-model-tracking-envelope.md`. Lesson
for this entry's method: "the shipped config has margin" was asserted from a
bag recorded on a *different timing path* than the shipped one — margin
claims must name the code path they were measured on.

## Withdrawn claims

- *"The encoder overshoots the command — a command-side discontinuity from the
  retiming model or install continuity"* (this session's first framing): the
  negative `max_dev_value` is the u0−enc sign convention during a fast
  commanded descent; the command is a clean quintic and the plans are
  identical to the pre-SPE validated session.
- *"The plant chases at a hard ~0.63 rev/s clamp — something slashed vel_limit
  or gains"* (mid-analysis): an artifact of 0.1 s-grid sampling; the actual
  chase peaks 2.0–2.7 rev/s, matching 2026-07-16, and iq/vel signatures rule
  out a pushed-limit change (b6391c1's YAML diff touches only trajectory-node
  session limits and lean_gain — nothing that reaches the drives).

## Open Questions

- What physically drove today's +10–30 % lag at matched commands? Candidates:
  battery SOC (bus voltage is not recorded in the bag — worth adding to
  `/robot_state` before the accel-FF A/B), actuator temperature, first-moves
  stiction. Run 2's 6 % improvement over run 1 suggests warm-up is part of it.
- Leg 1 is the deepest lag both days and first to latch in every S4-era trip;
  the 2026-07-16 entry carries the same observation. If leg 1 keeps leading
  after accel FF lands, it deserves its own look (mechanical drag asymmetry?).

## Related

- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the lag
  mechanism, guard raise, and per-leg attribution this entry leans on.
- `logbook/2026-07-17-s4-closed-working-point-persisted.md` — the working
  point + lean persistence whose margin framing this entry corrects.
- `logbook/2026-07-17-shaped-planning-efficiency-implemented.md` — the SPE
  work exonerated here.
- `plans/parked/accel-ff-inertia.md` — the actual fix for the velocity
  deficit.
- `tests/hardware/session_phase4_ramp.md` — S4 protocol.
