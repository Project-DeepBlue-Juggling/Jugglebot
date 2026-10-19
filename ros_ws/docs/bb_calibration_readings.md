# Ball Butler calibration readings ledger

A running record of what every calibration sitting read, kept so that the
two open mechanisms (the 0.3 deg kappa step between sittings; the marker
constellation deforming at BB power events) can be tested against conditions
that were not recorded at the time: temperatures, BB power events, QTM
camera power and calibration state, room events. Narrative lives in the
logbook entries named at the bottom; this page is the table. Add a block
per sitting; never rewrite old rows.

**Status 2026-10-16:** kappa's level tracks ambient temperature across 23
sittings in two clusters (about two independent transitions; a correlation,
no controlled test yet; no resolvable slope inside either cluster); the Q3-Q6
pair deformation is set at BB power events, not by temperature. Both mechanisms are unidentified. See
§ Findings and § Hypotheses.

## How to add a sitting

1. Replay the bag(s): `python tools/probes/bb_calibration_replay.py --bags
   ~/Desktop/rosbags/<bag> --subset 1,2,4,6,7 -v` (per-sweep kappa, template
   residual, worst pair, shelf residual and heading, lag, epoch in
   `temp/probes/bb_calibration_replay/<run>/sweeps.csv` and `summary.md`).
2. ODrive temperatures around each sweep: `/robot_state` `motor_states[i]`
   `fet_temp` / `motor_temp` (legs 0-5, hand 6, BB pitch 7, BB hand 8). The
   probe that built the table is kept with its run: `temp/probes/
   bb_calibration_replay/20261016-221458/extract/probe_motor_temps.py` (point
   it at a run's `sweeps.csv`). The legs' `motor_temp` at a sitting's start is
   the room-temperature proxy (motors idle for hours before a calibration).
3. Record what the bag cannot: QTM camera power-on time and the wand
   calibration in use, the QTM camera temperatures (Camera Settings in QTM
   shows them), BB power events with times, the room (heater, door, sun),
   anything touched near BB or the shelf.
4. Append the row (and the per-event rows if BB was power-cycled inside the
   sitting), then the condition notes under § Sitting notes.

## Readings per sitting

Replay runs `20261016-181127` (2026-10-10 to 10-16 16:23) and
`20261016-221458` (10-16 20:31 and 22:01). Kappa and residuals are the
replay's (node and replay agree on every logged measurement but one, see
the entries). Temperatures are medians over the sweep windows (+/- 30 s);
`-` where the bag was not retained or the field is absent. Shelf heading is
in whatever world frame QTM had at the time (re-aligned 10-11, 10-12, 10-14
and 10-16 15:11); kappa is frame-invariant.

Note (2026-10-17): the 2026-10-16 22:01 row's refusals were under the 1.0 mm cap; the cap is 1.5 mm (advisory 1.0) from 2026-10-17.

| sitting (local) | sweeps acc/ref | kappa med (range) | epoch | template res med (max) | worst pair | shelf res | shelf heading | lag ms med | legs motor C | legs FET C | hand FET C | BB hand FET / motor C |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-10-10 00:24 | 0/7 (7 CONSTELLATION_) | - | - | - (-) | - | - | - | - | 22.5 | 31.9 | 33.5 | 29.3 / 27.6 |
| 2026-10-10 10:32 | 5/0 | - | - | 0.40 (0.41) | Q1-Q3 | - | - | -1.8 | 20.5 | 28.3 | 28.3 | 23.8 / 23.7 |
| 2026-10-10 12:26 | 10/0 | 179.332 (179.286..179.358) | K0 | 0.26 (0.37) | Q4-Q7 | 0.00 | -179.051 | 1.2 | 21.1 | 29.1 | 30.0 | 25.1 / 24.7 |
| 2026-10-10 13:46 | 8/0 | 179.334 (179.288..179.389) | K0 | 0.29 (0.41) | Q1-Q3 | 0.26 | -179.048 | 0.7 | 21.4 | 29.3 | 29.8 | 25.5 / 25.2 |
| 2026-10-10 14:11 | 3/0 | 179.396 (179.298..179.412) | K0 | 0.28 (0.30) | Q4-Q7 | 0.05 | -178.729 | -3.3 | 21.7 | 29.5 | 30.9 | 26.2 / 26.0 |
| 2026-10-10 14:13 | 4/0 | 179.323 (179.303..179.333) | K0 | 0.27 (0.31) | Q3-Q6 | 0.06 | -178.730 | -2.4 | 21.7 | 30.0 | 31.6 | 26.8 / 26.2 |
| 2026-10-10 14:49 | 1/0 | 179.298 (179.298..179.298) | K0 | 0.40 (0.40) | Q3-Q6 | 0.07 | -178.728 | -4.2 | 22.1 | 29.7 | 32.0 | - / - |
| 2026-10-10 19:23 | 1/0 | 179.270 (179.270..179.270) | K0 | 0.47 (0.47) | Q3-Q6 | 0.14 | -178.705 | -0.2 | 22.3 | 29.8 | 30.0 | 25.5 / 26.0 |
| 2026-10-10 23:32 | 1/0 | 179.224 (179.224..179.224) | K0 | 0.49 (0.49) | Q3-Q6 | 0.11 | -178.693 | 0.8 | 21.0 | 28.6 | 28.9 | - / - |
| 2026-10-11 11:28 | 1/0 | 179.233 (179.233..179.233) | K0 | 0.70 (0.70) | Q5-Q7 | 0.19 | -177.520 | 2.0 | 19.9 | 27.5 | 28.8 | 23.1 / 23.3 |
| 2026-10-11 15:14 | 2/0 | 179.316 (179.304..179.327) | K0 | 0.36 (0.44) | Q4-Q6 | 0.19 | -177.517 | -9.0 | 21.6 | 29.4 | 31.0 | 25.5 / 25.6 |
| 2026-10-11 22:16 | 1/0 | 179.311 (179.311..179.311) | K0 | 0.39 (0.39) | Q1-Q4 | 0.11 | -177.307 | 0.3 | 22.9 | 30.7 | 31.7 | 25.9 / 25.9 |
| 2026-10-12 23:24 | 2/0 | 179.269 (179.265..179.273) | K0 | 0.32 (0.32) | Q1-Q4 | 0.23 | -177.049 | 0.8 | 19.7 | 27.6 | 29.8 | 23.9 / 23.8 |
| 2026-10-13 00:18 | 1/0 | 179.271 (179.271..179.271) | K0 | 0.41 (0.41) | Q3-Q7 | 0.10 | -177.308 | -7.4 | 20.0 | 27.7 | 29.8 | - / - |
| 2026-10-13 22:00 | 6/0 | 179.554 (179.520..179.574) | K1 | 0.81 (0.87) | Q5-Q7 | 0.33 | -177.258 | -6.2 | 16.4 | 24.9 | 25.6 | 20.1 / 19.5 |
| 2026-10-13 22:29 | 8/0 | 179.598 (179.524..179.682) | K1 | 0.44 (0.49) | Q1-Q4 | 0.17 | -177.272 | -4.8 | 16.7 | 25.1 | 26.6 | 20.8 / 20.4 |
| 2026-10-13 23:13 | 4/0 | 179.569 (179.524..179.597) | K1 | 0.63 (0.76) | Q5-Q7 | 0.23 | -177.248 | -6.4 | 17.2 | 25.8 | 27.6 | 22.4 / 21.4 |
| 2026-10-14 20:54 | 6/1 (1 CIRCLE_CENTRES) | 179.628 (179.445..179.664) | K1 | 0.48 (0.52) | Q5-Q7 | 0.37 | -176.628 | 4.7 | 15.5 | 23.9 | 24.8 | 18.3 / 17.6 |
| 2026-10-14 21:27 | 6/0 | 179.639 (179.617..179.668) | K1 | 0.50 (0.54) | Q5-Q7 | 0.49 | -176.531 | 2.6 | 16.2 | 25.6 | 27.1 | 20.1 / 19.6 |
| 2026-10-14 22:00 | 1/0 | 179.594 (179.594..179.594) | K1 | 0.65 (0.65) | Q3-Q7 | 0.40 | -176.529 | 1.0 | 16.5 | 28.5 | 29.9 | 21.3 / 20.4 |
| 2026-10-16 10:51 | 2/0 | 179.569 (179.565..179.572) | K1 | 0.35 (0.35) | Q3-Q7 | 0.29 | -179.127 | 4.9 | 16.8 | 25.5 | 26.9 | 20.5 / 20.4 |
| 2026-10-16 15:08 | 7/0 | 179.262 (179.254..179.276) | K0 | 0.45 (0.46) | Q3-Q6 | 0.30 | -179.128 | 1.4 | 20.0 | 28.4 | 30.9 | 25.2 / 24.1 |
| 2026-10-16 16:10 | 1/0 | 179.339 (179.339..179.339) | K0 | 0.18 (0.18) | Q3-Q4 | 0.20 | 169.136 | -11.9 | 20.0 | 28.2 | 30.3 | 24.1 / 24.2 |
| 2026-10-16 16:23 | 1/0 | 179.242 (179.242..179.242) | K0 | 0.25 (0.25) | Q3-Q6 | 0.28 | 169.137 | -11.4 | 20.1 | 28.1 | 30.1 | 24.5 / 24.7 |
| 2026-10-16 20:31 | 6/4 (4 CONSTELLATION_) | 179.269 (179.220..179.379) | K0 | 0.30 (0.42) | Q4-Q7 | 0.31 | 169.151 | 1.2 | - | - | - | - / - |
| 2026-10-16 22:01 | 1/3 (3 CONSTELLATION_) | 179.273 (179.273..179.273) | K0 | 0.89 (0.89) | Q3-Q6 | 0.25 | 169.130 | -1.3 | 20.6 | 30.7 | 32.5 | 24.8 / 24.4 |

Per-event rows, 2026-10-16 (BB-only power cycles inside one launch; signed
Q3-Q6 pose-binned pair residual, same sign on both sweep legs in every sweep):

| event | sweeps | kappa | Q3-Q6 (mm) | note |
|---|---|---|---|---|
| launch 20:30:57, BB boot 20:31:02 | #1-#4 20:31-20:32 | 179.22, 179.23, 179.38, 179.27 | +0.09, +0.05, +0.05, +0.16 | residual 0.25-0.38, worst pair Q4-Q7 |
| BB off 72 s (20:32:49 to 20:34:01) | #5, #6 20:34 | 179.27, 179.27 | +0.33, +0.42 | rest yaw read 356.5 after the reboot (359.6 before) |
| BB off 85.6 min (20:34:57 to 22:00:31) | #7-#10 22:00-22:01 | 179.27, 179.26, 179.25, 179.23 (cap lifted) | +1.20, +1.20, +1.25, +1.14 | all refused CONSTELLATION_RESIDUAL; rest yaw 357.8 after the reboot |
| launch restart 22:01:50 (no BB power event) | #1-#4 22:02-22:03 | 179.22, 179.27, 179.37, 179.26 | +1.19, +0.89, +1.20, +1.23 | one accepted at 0.89; restart changed nothing |

Earlier the same day (one wand calibration from 10:28): 10:51 Q3-Q6 -0.07
(kappa 179.57, cold), 15:08 +0.44 to +0.46 over seven sweeps (179.26),
16:10 -0.05 (179.34), 16:23 worst pair Q3-Q6 at 0.25 (179.24). The state
is set at BB power-on and holds within a sitting.

## Findings (2026-10-16)

1. **Kappa's level correlates with ambient temperature, as a threshold (correlation only).** Every
   sitting at a legs-motor temperature of 15.5 to 17.2 C read 179.55 to
   179.64 (10-13 evening, 10-14 evening, 10-16 10:51); every sitting at 19.7
   to 22.9 C read 179.22 to 179.40 (10-10, 10-11, 10-12 23:24, 10-16 from
   15:08 on). No overlap, and no resolvable slope inside either cluster (15.5 vs 17.2 C
   both read about 179.6, the cold cluster drifting about -0.06 deg over 1.7 K
   inside the sitting scatter; 19.7 vs 22.9 C both read about 179.3). 23
   sittings, but one contiguous cold block, so about two independent
   transitions. The
   transition sits between 17.2 and 19.7 C. The BB hand ODrive's FET
   temperature says the same (18-22 C cold, 23-27 C warm). This is a better
   description of the step than "between sittings" (the boundaries that
   stepped were all cold/warm boundaries: the cold sittings were the
   step-ups, and 10-16 10:51 at 16.8 C after a cold night to 15:08 at 20.0 C
   was the step-down), but it is a correlation.
2. **The Q3-Q6 deformation is not thermal; it is set at BB power-on.** At
   20.0 to 20.6 C (measured at 15:08, 16:10, 16:23 and 22:01; the 20:31 and
   20:34 values are inferred from the neighbours, that bag is deleted) it read +0.44 (15:08), -0.05 (16:10), +0.10 (20:31),
   +0.35 (20:34, after a 72 s off) and +1.2 mm (22:00, after an 86 min
   off), each value holding for the whole sitting. The 72 s step came two
   seconds after the reboot. QTM's per-marker reconstruction residuals and
   the co-located shelf constellation (240 mm span, 244 mm from BB's axis by
   the replay's base origin and axis point, residual 0.20-0.31 mm across the
   day and 0.25-0.29 after the long off) did not change, and the error has the
   same sign on both sweep legs (so it is not a camera-timing artefact).
   The owner checked 2026-10-16: all seven markers are rigid on BB's base.
3. **Kappa does not follow the deformation.** Tonight the deformation ran
   -0.05 to +1.2 mm while kappa stayed at 179.27 +/- 0.05.

## Findings (2026-10-17)

4. **Template-residual cap evidence (decision: cap 1.5 mm, advisory 1.0 mm, owner, 2026-10-17).**
   Across 89 real sweeps kappa deviation from the sitting reference does not
   grow with the residual up to 1.25 mm (signed slope -0.022 +/- 0.016
   deg/mm). Injection into the real solver (1589 solves) gives a yaw bias
   linear in displacement, worst 0.155 deg per mm of residual (Q3 vertical),
   so the worst admitted bias at the cap is 0.23 deg. Vertical modes are
   invisible to the residual (Q5 -z: residual flat at 0.33 mm out to 3 mm
   displacement, bias 0.077 deg). Entry
   `2026-10-17-bb-calibration-residual-cap-1p5.md`.

5. **Robot frame measured in software (Phase 3, 2026-10-19, code landed, NOT flown).**
   From Phase 3 the Jugglebot (robot) frame is fitted by `mocap_node` from the
   labelled `Base - 1..7` markers and published as `robot_frame`; QTM is no
   longer re-aligned to the base. The sweep calibration and the shelf frame
   stay in QTM world, so rows above are unchanged. A shift of the Jugglebot
   frame in the world therefore no longer appears in this ledger as a QTM
   re-alignment. Entry `2026-10-19-bb-calibration-phase3-robot-frame.md`.

   2026-10-19 (second cut, code landed, NOT flown): the first cut flapped on its first sitting (the fit residual is 1.0-1.6 mm of per-configuration QTM reconstruction bias); the robot-frame fit is now yaw-only, drop-worst to 5 markers, cap 1.5 mm, a hold rule, no tilt guard. Entry `2026-10-19-robot-frame-fit-yaw-only-hold.md`.

   2026-10-19 14:24 (second cut FLOWN): locked 0.67 s, 7 markers, rms 0.87 mm, never UNKNOWN; hand move 48.5 mm / 14.5 deg tracked, hops 8/8 caught; robot-frame cap raised to 2.0 mm the same afternoon (move observability + settling refusal landed, NOT flown). Entry `2026-10-19-robot-frame-move-observability.md`.

## Hypotheses register

| # | mechanism | for | against | discriminator |
|---|---|---|---|---|
| H1 | BB yaw drive: temperature-dependent clearance or preload (gear mesh, bearing, coupling) puts the output on one of two flanks relative to the encoder | two flat levels, cold vs warm, no slope; the stage swings 2-3 deg when de-energised (something loads it) | an unpowered swing and re-engagement did not flip it at 20 C | read BB's yaw at a mechanical reference (a hard limit) cold and warm within one day: a 0.3 deg shift at the stop is BB-side |
| H2 | AS5047P magnet/chip geometry moving with temperature (a lateral magnet offset of order 0.1 mm gives angle errors of order tenths of a degree; datasheet-order figure, not measured here) | thermal, BB-side, no markers involved | a continuous mechanism would show a slope inside clusters; none seen | same as H1; also compare encoder reading vs a dial/protractor on the stage cold and warm |
| H3 | QTM camera or mount geometry with room temperature, biasing BB's small constellation differently from the shelf's | thermal; cameras and mounts do move with temperature | the shelf (same span, 244 mm away) heading held to 0.001-0.002 deg across the 10:51 to 15:08 step; a global drift cancels in kappa by construction | a rigid dummy constellation with BB-like spans fixed next to BB: its fitted yaw against the shelf heading moving with temperature is QTM-side |
| H4 | Partial occlusion or merging of Q3 or Q6 by a BB part whose position is set at power-on (hand homing, ball, hopper) | power-event-locked, marker-specific, constant through a sweep, shelf untouched, both legs same sign | the ball was in hand at both 16:23 (0.25 mm) and 22:01 (1.2 mm) (`/bb/heartbeat` `ball_in_hand`, read 2026-10-16) | QTM 2D views of Q3 and Q6 per camera before and after a BB power cycle; or cover/move the candidate part and re-sweep without a power event |
| H5 | A sub-assembly of BB's base with play, re-seated by the de-energised swing at each power event (Q6, Q1 and Q7 move together; Q3 moves against all) | pattern of which pairs changed; power-event-locked | the owner found the markers rigid on the base (the base's own joints were not the question asked) | push on the base parts carrying Q3 and Q6 by hand and watch the live pair residual; mark and photograph the parts across a power cycle |
| H6 | Cable harness load on the platform changing with each power event | the stage swings 2-3 deg unpowered, so the harness settles differently each time | no direct evidence | observe the harness across a power cycle; re-route and repeat |
| H7 | Handling at BB's switch bumping something | both tonight's events had the owner at the switch | no evidence either way for the 15:08 state (+0.44); markers rigid (owner, 2026-10-16) | switch BB from the PSU without touching it |

Retired: a marker or post physically moved (owner's check 2026-10-16);
labelling or template artefact and the firmware (Phase 2 entry); camera
timing (direction split above); reconstruction noise (QTM residuals).

## Sitting notes

- 2026-10-09: template built (7-sweep bag 18:56); session A/B wand calibration.
- 2026-10-13 to 10-14 evenings: cold room (legs 15.5-17.2 C); the 0.3 deg step up.
- 2026-10-16 10:28: QTM re-wanded (previous wand calibration 562 h old). 10:51
  cold (16.8 C), kappa 179.57. 15:08 warm (20.0 C), 179.26; world frame
  re-aligned 15:11. 16:10 and 16:23: one sweep each, BB power-cycled before
  each. 20:30-22:03: the BB-only power-cycle A/B (per-event rows above);
  QTM, frame and room untouched; the 20:30 bag deleted after extraction,
  its calibration windows under the run's `extract/`.
- 2026-10-17: no sitting. Residual cap raised to 1.5 mm (advisory 1.0). The next
  sitting's first sweep reads whether the Q3-Q6 deformation persists; a residual
  between 1.0 and 1.5 now arrives as an accepted sweep with a TEMPLATE_RESIDUAL
  advisory, so read the outcome line, not a refusal.

## Entries

- `logbook/2026-10-14-bb-calibration-one-attempt-investigation.md` (census)
- `logbook/2026-10-15-bb-calibration-phase0-characterise.md` (bands)
- `logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md` (gate, first two-state reading)
- `logbook/2026-10-16-bb-calibration-phase2-template-residual.md` (the step is at BB, not labelling)
- `logbook/2026-10-16-bb-calibration-power-cycle-ab.md` (not a power-up state; the deformation; this ledger's first rows)
- `logbook/2026-10-17-bb-calibration-residual-cap-1p5.md` (cap 1.5 / advisory 1.0 on measured evidence)
- plan: `plans/archived/bb-calibration-one-attempt.md`
