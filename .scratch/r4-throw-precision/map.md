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

## Not yet specified

- **The fixes for the self-toss landing error and the hop overshoot.** Their shape depends entirely on where tickets 01/02 put the error: commanded plan, plant tracking (the legs/hand not following the plan at release), ball–cup release physics, the frame/levelling transform, or the learner feeding back a bias.
- **The re-aim refusals** (LIMIT_JERK 164k–202k mm/s³ and HAND_LIMIT_C2 on the splice). Should re-aim exist at all once throws are precise, or does it just need a smaller or jerk-aware authority? Revisit after 01.
- **Learner memory.** Rows accumulated through sitting 3 (5 → 10 → 11). Is it helping or chasing a bias? Quarantine question for the next sitting.
- **Late seats** (+0.07..+0.30 s on self-toss, +0.65 s on the hop catch). Is this normal seat latency or a timing error?
- **The R4 gate sitting itself**, once fixes are in.

## Out of scope

<!-- work ruled beyond the destination -->
