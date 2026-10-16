# BB calibration Phase 1 - dress-rehearsal runsheet

Plan `plans/active/bb-calibration-one-attempt.md` Phase 1; entry `logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md`. Nothing is powered beyond BB; every refusal and advisory is reported, none stops the row.

## Pre-flight

1. Merge `bb-calibration` into `skill-stack`.
2. `cd ros_ws && colcon build --packages-select jugglebot`
3. Restart the launch; bag ON.

## Rehearsal rows

| # | action | expect |
|---|---|---|
| R1 | read the console after launch | the startup line appears once and reads the migrated state (expect `pool 8 records in 1 epochs (current K0 median 179.59...)` from the live file: 8 schema-1 records at pin 0.029, kappa 179.57-179.68, all post-10-13, migrated into K0); a `.bak` is written on the first accepted sweep |
| R2 | one sweep with the shelf markers visible | ACCEPTED; the outcome line carries kappa + epoch + template residual + lag + sweep suffix. Record kappa, template residual, flags. THIS IS THE PIN READING: template residual near 0.26 mm and kappa off 179.59 = reconstruction owned the step; kappa on 179.59 with the residual down = something at the shelf |
| R3 | one sweep with the shelf markers covered | ACCEPTED with the "shelf base not seen: kappa not pooled" line |
| R4 (optional) | deliberate QTM re-alignment, then a sweep | ACCEPTED with QTM_FRAME_SHIFT and no YAW_SHIFT judgement |

## Acceptance log (plan acceptance 2: first sweep of ten consecutive sittings ACCEPTED)

| # | sitting date | first-sweep verdict | kappa | epoch | flags |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |
| 6 | | | | | |
| 7 | | | | | |
| 8 | | | | | |
| 9 | | | | | |
| 10 | | | | | |
