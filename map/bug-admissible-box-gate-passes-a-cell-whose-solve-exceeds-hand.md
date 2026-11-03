---
id: JB-120
kind: bug
title: 'A BUMP of catch N leaves a release seed catch N+1 cannot plan from: HAND_LIMIT_ACC 3910-4097 vs 3900 at the next first solve'
status: in_progress
created: '2026-10-31'
updated: '2026-11-03T03:32:20Z'
owners:
- jugglebot-50
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
parent: two-ball-oval
filed_by: claude-books-2026-10-31
---

The admissible-box gate reports a Columns Ball 2 catch as `inside box` while the realised solve is refused on Hand acceleration, so the attempt ends at the first install.

## Evidence (2026-10-31, goal 4, Columns with a Feed, launch 13-14-00)

- Install refused: `REJECTED_CYCLE_INFEASIBLE(HAND_LIMIT_ACC: peak hand acceleration 4112.5 rev/s^2 > 3900.0)` at skill 13, the CATCH of Ball 2, after 12 of 13 caught.
- The same second the gate printed `BOX-EXPECTED-LANDING skill 13: u=(-16.6, -20.0) mm ... inside box x[-2.0,+1.0] y[-20.0,+8.0]`.
- The learner command for Ball 2 y sat at -20.0 mm, the box edge, on every Ball 2 catch of that goal (memory 2112+ rows, k=16).
- Gate and solver disagree by about 5 percent at the y = -20 edge. Zero such refusals in the 10-28 and 10-30 logs; the cell was 50/50 on 10-30.
- The 10-30 prep entry (C1) already put this cell within 2-6 percent of the Hand limit at hold 0.100.

## Fix candidates

1. Shrink the box y lower edge from -20 toward -15 for the Columns Ball 2 cell.
2. Add a `CATCH-AIM-FALLBACK` for `HAND_LIMIT_ACC`, the same shape as the existing `LIMIT_VEL` fallback to the unlearned aim.
3. Replay u = (-16.6, -20.0) offline with the box tool first to see which u produces the peak.

## Night Shift replay (2026-11-01)

Offline replay through the production `plan_segment` (flown tree `48bf4fcb^` and HEAD, hold 0.100 and 0, the sweep's seed and a live-like seed, catch aim 0 and -20 mm y, throw u up to (-16.6, -20) mm): **no case reaches 3900 rev/s^2**; the highest is 91 percent, the live 4112.5 is 14 points above it.

The lateral inputs move the peak by at most 0.7 points, the seed by 12. Fix candidate 1 (shrink the y edge) is withdrawn. The input left is the live seed (Ball 1's installed plan at k_s 293 after its re-aim and bump), which is not logged.

- Fixed on the branch: every goal-4 catch dispatched `source=schedule:learned`, so the unit-A2 fallback to y_d should have run and did not, because `install_segment` returns the bare check code `HAND_LIMIT_ACC` and the classifier only knew `LIMIT_*` / `INFEASIBLE`. The branch adds the `HAND_LIMIT_` prefix (one more solve, only on a refusal). The replay says y_d alone is unlikely to rescue this cell.
- Two sweep gaps, real but not shown causal: `_within_margin` applies the 0.9 margin to the legs only (the Hand is certified at 100 percent; this cell sits at 90 in the sweep's shape), and the box bounds the expected landing while the sweep certified command = landing (the live u x = -16.6 mm is outside the box x [-2, +1] and the sweep's shape refuses LIMIT_VEL there).
- Today's defaults (hold 0, stroke top +16 mm) put every replayed case at 66-79 percent.

Entry: logbook/2026-11-01-jb120-box-gate-vs-hand-limit-acc-replay.md.

## Sitting 2026-11-03 flown

Flown 2026-11-03 (passed on the Hand criterion, see Progress); renamed from `## Sitting` so the generator, which lists every open hardware Item's exact `## Sitting` section, does not offer it again. The structural bump fix will write a fresh `## Sitting`.

- setup: armed, bb-fed
- why: re-fly with the bump brake-seed fix (2026-11-02 debrief): the 2026-11-02 refusals all followed an inward bump of 22 mm or more and replay under 3700 rev/s^2 with the fix.
- do:
  - Fly Columns, Sep. 125, Apex 0.95, Throws 50, twice.
- pass: no END HAND_LIMIT_ACC and no refused first install in either 50.
- record: per-catch peak Hand acceleration from the install lines and the learner y for Ball 2.
- minutes: 15

## Sitting 2026-11-01 findings

- 7 of 7 refusals are the first install of a catch seeded from a BUMPED predecessor (0 of about 59 unbumped seeds).
- Offline `plan_segment` plus `apply_catch_bump` reproduces the peaks within 3 pct: the peak sits on span 1 of the post-release hold.
- The branch fallback was not flown and could not have helped; the box is blind to bumped seeds.
- Branch `night-shift/2026-11-01-JB-120` stays held (the limit decision is below).

**Owner decision 2026-11-01 (debrief).** HAND_LIMIT_ACC 3900 -> 4500 rev/s^2 and hand_curr_limit_a 50 -> 60 A (4500 needs ~57 A at the modelled Kt/J; owner open to 70-80 A later, heating manageable; the hand ODrive hard max is 70 A); box re-swept under the new limits.

The structural fix is FILED here as the next step, not built: the bump decision also plans the next catch and declines or shrinks the bump if that solve refuses; late re-sends fall under the same rule. JB-117 bench hold-and-step measures the real authority.

## Sitting result (2026-11-02)

- verdict: fail
- notes:
  - First attempt got to pretty much exactly 50 throws before hitting " ERROR columns ENDED (HAND_LIMIT_ACC) at the CATCH: hand acceleration 4509.26 > 4500 rev/s^2 · 49/51 caught · apex 0.93–1.02 m · worst landing 41 mm · release +6 ms..+21 ms"
  - Second attempt ended after ~30 throws with leg vel limit: ERROR columns ENDED (LIMIT_VEL) at the CATCH: leg velocity 355.33 > 350 mm/s (leg 1 @ +0.175 s)
  - A few other attempts ended with collisions/missed throws and leg LIMIT_VEL errors. Tehcnically this was a failure due to the first LIMIT_ACC fire, but I think this was much more successful than previously!

## Sitting result (2026-11-03)

- verdict: pass
- notes: Ran a few attempts of the described columns pattern. Some ended prematurely due to messy throws, collisions, LIMIT_VEL errors, and GUARD_LATCHED errors, but none due to HAND_LIMIT_ACC

## Structural fix landed unwired (2026-11-03)

- I-CATCH-7 in `bump.py::bump_with_next_catch_gate`: a Bump is applied at the largest of D, D/2, D/4, D/8 whose seed the next catch's gate accepts, else declined `BUMP-NEXT-CATCH`; a shrink is logged `BUMP-SHRUNK`, never silent.
- No cheap gate exists: only the next catch's own solve (33-93 ms) predicts the Hand peak, too slow for the 25-50 ms bump window, so no live caller passes a gate yet and the live path is unchanged.
- Owner call 2026-11-03: land unwired. Offline, every bumped seed up to 30 mm now plans at or under 82 pct of the 4500 limit.
- Trigger to wire it (option B, an offline bump envelope per box cell): a HAND_LIMIT_ACC after a Bump at the 4500 limit.
- The fallback branch night-shift/2026-11-01-JB-120 was dropped per JB-152. Logbook 2026-11-03-jb120-bump-gated-by-the-next-catch.

## Progress

- 2026-11-01: Night Shift 2026-11-01: held on night-shift/2026-11-01-JB-120: hardware robot; audit open on HAND_LIMIT_C2 admitted by the new HAND_LIMIT_ prefix and no both-refused test; unblocked by the Sitting in the section below plus the owner's call on the C2 finding (detail below)
- 2026-11-01: Sitting: 6 Columns ENDs HAND_LIMIT_ACC 3910.6-4096.8 vs 3900; 7 of 7 refusals follow a BUMPED predecessor (0 of ~59 unbumped); reproduced offline within 3 pct. Branch fallback not flown, could not help. Owner keeps 3900; branch stays held (detail below)
- 2026-11-01: Owner decision (debrief): HAND_LIMIT_ACC 3900 -> 4500 rev/s^2, hand_curr_limit_a 50 -> 60 A, box re-swept; the structural bump fix (plan the next catch at bump time) stays FILED here, not built (detail in the findings section).
- 2026-11-02: Failed at the Sitting of 2026-11-02: HAND_LIMIT_ACC 4509 at 49/51 after a 30.5 mm bump; all 3 refusals of both Sittings follow the 3 largest inward bumps (22.2-30.5 mm), 0 of 329 unbumped. New cause found and fixed: the bump kept the pre-bump C-HAND-5 brake seed (offline 4696 -> 3407). Structural fix stays filed.
- 2026-11-03: Passed at the Sitting of 2026-11-03 on the Hand criterion: 0 HAND_LIMIT_ACC after 71 bumps, peak Hand acc 3794 rev/s^2 = 84% of 4500 (L2532). 15 leg LIMIT_VEL/JERK refusals and 3 Hand faults in the block belong to JB-24 and JB-147. Held branch dropped (JB-152 option 3); Item stays open for the structural bump fix.
- 2026-11-03: Structural fix landed UNWIRED (I-CATCH-7, bump_with_next_catch_gate); owner call: land unwired, wire option B if a HAND_LIMIT_ACC after a Bump fires at 4500; fallback branch dropped per JB-152 (detail below)
