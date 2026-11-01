---
id: JB-139
kind: bug
title: Ball 2 learner command pinned at the (-20,-20) mm clamp corner in 37 of 40 straight Columns rows (BOX-EXPECTED-LANDING-UNMET u=(-20,-20))
status: inbox
created: '2026-11-01'
updated: '2026-11-01T17:59:48Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: learner
blocked_by: []
links:
- map/pathway-learner-generalisation.md
filed_by: claude
---

Found at the 2026-11-01 Sitting: the Ball 2 learner command sits at the (-20, -20) mm clamp corner in 37 of 40 straight Columns rows (BOX-EXPECTED-LANDING-UNMET u=(-20,-20)).

## Findings

- u1 (y) = -0.020 in 37/40 rows, u0 (x) = -0.020 in 13/40, both pinned in 12/40.
- It does not predict the HAND_LIMIT_ACC ends (JB-120): lateral u moves the Hand peak by under 1 point.
- Relevant to the learner generalisation work (JB-38).

## Findings (2026-11-02)

Night Shift 2026-11-02 replayed the committed learner memory (`temp/learn/jugglebot/memory.csv`, the 2026-11-01 straight Columns rows: 40 at the -62.5 mm site, Ball 2; 33 at +62.5 mm, Ball 1) through the production `learner.command` / `local_model` with the default `LearnerConfig`. The pinned corner is the clamp doing its job against a real, stable, site-specific plant bias; the fit is not distorting the command.

| Site / ball | n | landing - target x (mean, sd) | y (mean, sd) | mean u0 | mean u1 | u1 pinned | implied plant bias (x, y) |
|---|---|---|---|---|---|---|---|
| +62.5 mm (Ball 1) | 33 | -1.3, 15.5 | +0.5, 17.5 | -4.4 | -10.6 | 0 % | +3.1, +11.1 |
| -62.5 mm (Ball 2) | 40 | -0.0, 21.7 | +6.7, 18.8 | -15.6 | -19.3 | 90 % | +15.5, +25.9 |

- The Ball 2 site lands about +26 mm in y and +16 mm in x for a zero command (landing minus command, the fit's slope D being 0.97 to 1.01 on the diagonal). The y bias exceeds the 20 mm authority by about 6 mm, so the clamp binds and a +6.7 mm mean residual is left, well inside the 19 mm scatter.
- The unclamped y command is -25 to -27 mm (5 to 7 mm past the clamp); in x it is -14 to -18 mm, inside authority, pinned only transiently (13 of 40 rows). At u = (-20, -20) the model predicts a landing of about (-2.5, +5.0) mm, so the unmet amount is small.
- Not new with hold 0: the -62.5 mm site's y bias is +13 to +27 mm in every Sitting since 2026-10-19 (10-19 +20.1, 10-22 +21.6, 10-27 +13.2, 10-28 +13.0, 10-30 +27.3 on 8 rows, 10-31 +18.5, 11-01 +25.9), and the pinned fraction rises with it (0 % at +13, 90 to 100 % at +26 to +27). Ball 1's site shows +6 to +20 mm and never pins.
- Pooling across sites is ruled out: rows are keyed by the release site and the 10 mm kernel keeps the two sites apart. The residual does not predict the HAND_LIMIT_ACC ends (JB-120).

What would settle it: with a fresh `plant_id` the first Ball 2 commands are the identity prior, so the first landings show the raw bias directly; if they average +20 to +32 mm in y the bias is in the plant (feed or seat geometry at that site, a JB-38 question), not the fit. Do not raise the authority on this evidence alone: the residual is small and the question is why the bias is site-specific.

## Sitting

- setup: armed, bb-fed
- why: confirm the +26 mm y landing bias at the -62.5 mm (Ball 2) site is in the plant, not the fit, by reading the identity-prior landings of a fresh memory.
- do:
  - Fly straight Columns, Sep. 125, Apex 0.95, Throws 10, with a FRESH `plant_id` (a name no memory file has) so the first commands are the identity prior.
  - Read the Ball 2 landing y per throw from the install lines (or the new memory rows).
- pass: the first 5 Ball 2 landings average +20 to +32 mm in y (the bias is real and stable); outside that band the fit, not the plant, is the next question.
- record: per throw the Ball 2 landing (x, y) mm and the command u; the plant_id used.
- minutes: 10
