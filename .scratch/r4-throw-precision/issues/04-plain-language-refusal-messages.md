# Plain-language refusal messages

Type: task
Mode: AFK
Status: resolved
Blocked by: none

## Question

The owner finds the refusals cryptic. Rewrite them in plain operator language: say what happened, why it matters physically, and what to do. Keep the machine-readable code prefix, and keep the numbers.

1. `CATCH_AXIS` (unified_cycle): currently "held-axis catch: the seed's position is (5.681, -0.370) mm off the axis line through the touch-down ... kappa·(z − site_z)". It should read along the lines of: "the platform isn't lined up for this catch — its resting position is 5.7 mm to the side of the line the cup must travel along to meet the ball (allowed: 0.1 mm)".
2. The admissible-box refusal (`skill_node.py:2317`, `executor.py:1291`) lists only the apex bands. When the refusal is really the site positions (a non-default `separation_mm`; the boxes were swept for ±125 mm), it must say so. This is the owner-reported 0.95 m hop refusal at sitting 3.

Sweep the other `REJECTED_CYCLE_INFEASIBLE` reasons for the same jargon, but only rewrite the ones an operator actually sees. Tests that match message text must be updated, and a test must pin the separation case.

## Answer

(2026-09-29.) Implemented:
- The CATCH_AXIS seed-offset message in `cup_cycle.py` is rewritten in plain language and now reports the offset's magnitude.
- A new `admissible.describe_miss()` is used by both box refusals (`skill_node`, `executor`). It says in plain words whether the apex is out of range or the sites don't match the swept separation (the sitting-3 0.95 m case).
- New test `test_hop_refusal_names_the_separation_mismatch_not_just_the_apex` failed before and passes after.

The `cup_cycle.py` edit changes the admissible gate_hash, so the box must be re-swept; this is batched with the other gated edits. In the same gated batch, the remaining jargon-heavy planner refusals were also rewritten: the other CATCH_AXIS variants, CATCH_TOO_EARLY (×3), CUP_CONTACT_ACC, SINGULAR (×2), CATCH_RUNWAY, and the feasibility Jacobian-condition message (×5 sites). Codes and numbers are unchanged. One was missed and is carried: `_solve_qp`'s "unbounded dual step" message, which is gated and would cost another re-sweep.
