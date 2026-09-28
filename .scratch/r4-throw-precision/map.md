# Map: R4 throw precision (self-toss, hop, reload catch)

Label: wayfinder:map
Charted: 2026-09-29, after R4 sitting 3 (2026-09-28 23:55; bag `~/Desktop/rosbags/2026-09-28_23-55-19`, log `temp/logs/skills_r4_20260928_2355.log`).

## Destination

The R4 hardware gate (`tests/hardware/session_skills_r4.md`) is MET on the robot:
- self-toss lands within ±10 mm (1σ) of its scheduled landing **with in-flight re-aim OFF**, and at least 95 % of balls are caught;
- the hop throws land on the far site (no systematic +x overshoot);
- the Ball Butler reload catch actually moves the hand to receive the ball.

## Notes

- **This map carries execution (owner, 2026-09-29).** Tickets may land fixes, not just decisions. Plan § 0 of `plans/active/two-ball-skill-stack.md` stays normative, and every CLAUDE.md workflow rule binds: probe before writing tests, fail-before/pass-after, `./run_tests.sh` per commit and `--full` before a sitting, a logbook entry with (date, command, result) triples, commit + push in the same response.
- **Diagnose from bags before any sitting.** Offline analysis narrows the hypotheses. The owner then runs ONE diagnostic sitting (about 45 min) that Claude designs.
- Owner's physical read of the hop (2026-09-29): the ball leaves the cup cleanly. To the eye it looks like a **tilt/orientation** problem at release, but the throw is too quick to see. Treat this as load-bearing.
- Owner's premise on self-toss: a well-positioned platform should make the throws repeatable enough that the 20 mm lateral re-aim is not needed. The target of ±10 mm is believed achievable.
- Agents: Sonnet by default, and Opus only for the bag diagnosis (hardware diagnosis). Keep each agent under about 80 calls, and write handoffs to the scratchpad.
- When the map closes, fold it into a logbook entry.

## Decisions so far

<!-- one line per resolved ticket: [title](issues/NN-slug.md): gist -->

- [Self-toss landing error: bias or scatter, and which channel?](issues/01-self-toss-landing-error-decomposition.md): SCATTER (1σ 16/23 mm on rest throws), entering as ball launch velocity; the platform explains little. The owner's "well-positioned platform" premise does not hold; about 17/22 mm is release-side (ball–cup). The learner's lateral command wanders on noise.
- [Hop +x overshoot: what is the cup doing at the moment of the throw?](issues/02-hop-plus-x-overshoot-at-release.md): tilt is correct. The platform is still sliding at about +100 mm/s at separation (leg lag 25–50 ms) and the ball inherits it (slope about 1). The post-release hold could not act. Fix: a PRE-release hold (0.100 s), now implemented.
- [Reload catch refused: the resting pose is off the catch line](issues/03-reload-catch-seed-off-the-axis-line.md): the CATCH was dispatched 225 ms before the pre-tilt REST ended and was seeded mid-slew. Fixed in `compile_reload` timing; reload sim gate 5/5.
- [Plain-language refusal messages](issues/04-plain-language-refusal-messages.md): CATCH_AXIS plus the separation-aware box refusal, then all gated planner refusals rewritten.
- [Design the diagnostic sitting](issues/05-design-the-diagnostic-sitting.md): `tests/hardware/session_skills_r4_diag.md` covers the hop hold A/B, open-loop self-toss singles, a dwell 0.3/0.6 A/B on carried chains, and reload; lateral learning frozen, memory cold, no cup markers.

## Not yet specified

- **The self-toss release-side residual (17/22 mm).** Is it the ball moving in the cup, sideways give in the hand carriage, or cup-lip contact? Offline ([result_release_side](evidence/result_release_side.md)): the ball is tracked in the cup at 152 Hz and drifts only 2–6 mm over the last 150 ms, so the kick is at separation. The lateral velocity σ is flat from 0.6 to 0.9 m (weak n), which leans toward a fixed-size kick. Diag sitting B2 decides. The fix shape (cup geometry, ball seating, stiffer carriage, a sharper separation) waits on the diagnostic sitting's instrumented block.
- **No fallback on a refused CATCH.** The hand sits parked at the bottom of its stroke for an inbound ball. Should a refused catch fall back to a receive-height REST?
- **The `fresh`-predicate dead zone `[end_s − lead, end_s)`.** The node seeds from a mid-slew commanded state there. It is closed for the reload, but THROW 0 after the opening REST also dispatches into it (benign on-axis). Is there a class fix at the node?
- **The early tracker fit reads about 9 mm short in x**, which drives spurious re-aims.

- **The re-aim refusals** (LIMIT_JERK 164k–202k mm/s³ and HAND_LIMIT_C2 on the splice). Should re-aim exist at all once throws are precise, or does it just need a smaller or jerk-aware authority? Revisit after 01.
- **Learner memory.** Apex learning works (the hand runs 6–8 % fast, and the factor drifts between sittings); lateral learning wanders on noise. Freeze lateral learning? (Put to the owner in ticket 05.)
- **Late seats** (+0.07..+0.30 s on self-toss, +0.65 s on the hop catch). Is this normal seat latency or a timing error?
- **The R4 gate sitting itself**, once fixes are in.
- **A catch that may start while the platform is still settling** (owner, 2026-09-29): it would finish the receive tilt inside the catch window, then hold the line. That is a new catch shape, worth it only if the reload timing gets tight. Today's fix starts the pre-tilt 225 ms earlier instead, against more than 1 s of BB delay slack.
- **Hop at a smaller separation** (owner suggestion): the next discriminator if the pre-release hold fails its deciding number. It needs its own box sweep.

## Out of scope

<!-- work ruled beyond the destination -->

- **Columns (two-ball) under the pre-release hold.** The re-swept box (2026-09-29, gate `ad36fa53aab2`) admits NO columns throw: the sweep's columns THROW cell (0.3 s from rest) cannot fit a 100 ms hold (0 → OK, 0.05 → MARGIN, 0.1 → INFEASIBLE). The real chained columns pattern still passes the sim gate (20/20) with the hold on. Columns is not in the R4 destination, but it IS R5 (two-ball columns), so this is carried to R5 in the plan. Fly it with `pre_release_hold_s:=0` until its sweep cell or dwell is revisited.
