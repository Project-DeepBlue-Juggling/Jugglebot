# Design the diagnostic sitting

Type: grilling
Mode: HITL
Status: resolved
Blocked by: 01, 02, 03

## Question

Given what the bag analysis could not settle, what exactly should the owner run in one ~45 min sitting to confirm or kill the surviving hypotheses?

Candidates:
- self-toss with in-flight re-aim OFF and learner memory quarantined (the true landing spread);
- hops at more than one lateral distance or apex (does the overshoot scale with lateral speed?);
- a mocap marker set on the cup/platform to measure the realised tilt at release;
- a reload with the hand actually rising.

Output: a runsheet section with a hypothesis per step and the number that decides it.

## Answer

(2026-09-29, with the owner.) The runsheet is [`tests/hardware/session_skills_r4_diag.md`](../../../tests/hardware/session_skills_r4_diag.md). It has four blocks, each with a deciding number:
- **A. Hop hold A/B:** 5 hops on, then 5 off, using the live `pre_release_hold_s`. The owner contests the pre-release mechanism, and this block decides it.
- **B. 12 open-loop self-toss singles:** `catch_aim_source schedule`, `catch_resend_max 0`.
- **C. Carried chains, dwell 0.3 s against 0.6 s:** the owner's settling hypothesis.
- **D. 3 reloads.**

Owner decisions:
- lateral learning frozen (authority 0) for the sitting;
- cold memory (a fresh `plant_id`);
- no cup marker body for now (hard to fit), so offline release-side discriminators run instead;
- hand-centring is dropped, because the ball settles by itself;
- a smaller hop separation is deferred as the next discriminator if A fails.
