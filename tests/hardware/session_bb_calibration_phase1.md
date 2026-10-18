# BB calibration Phase 1 - dress-rehearsal runsheet

Plan `plans/active/bb-calibration-one-attempt.md` Phase 1; entry `logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md`. Nothing is powered beyond BB; every refusal and advisory is reported, none stops the row.

## Pre-flight

1. Merge `bb-calibration` into `skill-stack`.
2. `cd ros_ws && colcon build --packages-select jugglebot`
3. Restart the launch; bag ON.

## Rehearsal rows

| # | action | expect | result |
|---|---|---|---|
| R1 | read the console after launch | the startup line appears once and reads the migrated state (expect `pool 8 records in 1 epochs (current K0 median 179.59...)` from the live file: 8 schema-1 records at pin 0.029, kappa 179.57-179.68, all post-10-13, migrated into K0); a `.bak` is written on the first accepted sweep | **Result: PASS 2026-10-16: `BB calibration state: template bb_marker_template.json (7 markers), base model on, pool 8 records in 1 epochs (current K0 median 179.59 n 8), last accepted 2026-10-15T23:51:55 yaw +0.46, yaw source auto` once; `.bak` (schema 1, 8 records) written at 15:08:44** |
| R2 | one sweep with the shelf markers visible | ACCEPTED; the outcome line carries kappa + epoch + template residual + lag + sweep suffix. Record kappa, template residual, flags. THIS IS THE PIN READING: template residual near 0.26 mm and kappa off 179.59 = reconstruction owned the step; kappa on 179.59 with the residual down = something at the shelf | **Result: PASS (format): `BB calibrated: yaw +0.10 +-0.04 ... template 0.44 mm, lag +2.0 ms · kappa 179.23 (epoch K1 opened, n 1), p_b d0.7 mm · ADVISORY: yaw -0.36 deg vs the last sweep (advisory > 0.20); kappa -0.36 deg from epoch K0: opened epoch K1`. Pin reading: kappa off 179.59 but residual 0.44 not 0.26; the re-wand was NOT the cause (10:51 post-re-wand sweeps still read kappa 179.57), see the Phase 1 entry** |
| R3 | one sweep with the shelf markers covered | ACCEPTED with the "shelf base not seen: kappa not pooled" line | **Result: PASS: two sweeps ACCEPTED `shelf base not seen: kappa not pooled` (template 0.44/0.45 mm, lag +0.5/+2.4 ms)** |
| R4 (RETIRED 2026-10-19: QTM is no longer re-aligned, the robot frame is measured in software (Phase 3); a deliberate re-alignment is no longer a procedure, and the Phase 3 hardware check (rehearsal row 37) replaces it) | deliberate QTM re-alignment, then a sweep | ACCEPTED with QTM_FRAME_SHIFT and no YAW_SHIFT judgement | **Result: PASS: ACCEPTED, `QTM frame moved since the last sweep (213.4 mm, -11.74 deg); world deltas are not comparable`, kappa 179.27 (K1 +0.01, n 4), no YAW_SHIFT judgement** |

## Acceptance log (plan acceptance 2: first sweep of ten consecutive sittings ACCEPTED)

| # | sitting date | first-sweep verdict | kappa | epoch | flags |
|---|---|---|---|---|---|
| 1 | 2026-10-16 15:08 | ACCEPTED, no override | 179.23 | K1 (opened) | KAPPA_EXCURSION, YAW_SHIFT (-0.36 deg); counts as 1 of 10 under the reworded acceptance (owner decision pending, see plan § 0) |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |
| 6 | | | | | |
| 7 | | | | | |
| 8 | | | | | |
| 9 | | | | | |
| 10 | | | | | |

Open owner questions (2026-10-16): (a) the acceptance-2 wording (plan § 0: reword so a legitimately opened epoch does not fail the band clause); (b) was BB powered off between 10:58 and 15:05 that day?
