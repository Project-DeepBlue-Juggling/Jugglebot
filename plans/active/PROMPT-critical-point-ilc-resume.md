# PROMPT — Resume the critical-point ILC arc (parked 2026-08-14)

Self-contained session prompt, per the `plans/active/` PROMPT convention:
exempt from the INDEX table; **DELETE this file when its Done-means list is
satisfied** — the arc lives in the logbook.

## Invocation

```bash
cd /home/jetson/Desktop/Jugglebot-ilc && claude
```

First message: *"Read plans/active/PROMPT-critical-point-ilc-resume.md and
resume the critical-point ILC arc."*

## What this arc is

Task-level iterative learning control on the throw/catch critical points
(after Suresh & Atkeson, arXiv:2602.21302): task error measured on hardware
(mined records, sensor-truth possession), sensitivities by finite
differences through the production planning chain, trust-region batch
updates re-validated by the real admission gates. The normative record is
`plans/active/critical-point-ilc.md` — it is CURRENT as of parking and is
the single source of truth; this prompt only orients and sequences.

## State at parking (2026-08-14)

- Branch `critical-point-ilc`, worktree `/home/jetson/Desktop/Jugglebot-ilc`
  (a git worktree of `/home/jetson/Desktop/Jugglebot` — never edit the main
  tree; parallel sessions own it). HEAD `dcd714e`, clean, pushed. Base
  `5e046cc`; gate G-3 merged `mvp-trajectory-bringup`@`e75badd` as `712bcee`.
- **Phases 0–2 are COMPLETE, audited, and DORMANT** behind
  `jugglebot_operational.toss_ilc_enabled: false` (byte-identical OFF,
  pinned by a pre-Phase-2 arithmetic oracle test). **Gate 1 is CLOSED**
  (owner, 2026-08-13): ±0.15 ILC speed authority, sizing memo approved —
  decisions recorded in the plan.
- Last full gate: `./run_tests.sh -q`, 2026-08-13, this worktree: **5340
  passed, 5 skipped, 0 failed in 244 s**.
- Session history (newest first), all with full Discussion sections:
  `logbook/2026-08-13-critical-point-ilc-e1-impl-and-phase2.md`,
  `logbook/2026-08-13-critical-point-ilc-phase1-and-e1.md`,
  `logbook/2026-08-12-critical-point-ilc-g3-and-phase0.md`,
  `logbook/2026-08-11-critical-point-ilc-plan-kickoff.md`.

## Read first, in order

1. `plans/active/critical-point-ilc.md` — the whole file (≈40 min well
   spent; every phase result, gate decision, correction and open item is
   in it, dated).
2. The four logbook entries above, or their `logbook/INDEX.md` rows for a
   fast pass.
3. `git log --oneline 5e046cc..HEAD` — ten commits tell the build story.

## Conventions this arc ran under (owner directives)

- **Opus agents for all actual work** (implementation, audit, fixes);
  the orchestrating session does docs, review, git.
- Every unit closes: full gate + `/audit --unstaged` (Opus) before any
  multi-narrative commit; (date, command, result) triples on every count;
  commit and push in the same response; fetch-guard before push.
- Audits here have real teeth — every round found substantive issues
  (including in this arc's own headline numbers). Budget for a fix round.

## Next actions

**Desk-side (no hardware needed):**
- D1 — optional refresh-merge of `origin/mvp-trajectory-bringup` (the
  bridge-temporal arc advanced past `e75badd` after parking: S2/S3, FW 13 —
  read that branch's logbook INDEX first). Conflict class is the same as
  G-3: the two index tables, keep-both-rows.
- D2 — the parked `SIGMA_E` micro-decision (plan § Post-E-1 additions):
  arrival-dir noise 0.00238 → 0.00302 re-derivation; owner call, bounded
  cost either way.
- D3 — flagged hygiene: the aim-gain doc drift (3126.5/3126.64/3126.736 +
  secant-vs-derivative), and the ball radius into `hardware_config.yaml`
  (the E-1 probe had to assume 35 mm).
- D4 — draft Phase 3's pre-registered success criterion + abort signatures
  (plan § Phase 3 requires them FROZEN before the first A/B run).

**Operator (hardware sittings):**
- H1 — `colcon build --packages-select jugglebot` + relaunch before any
  session (node + config changed at Phase 2).
- H2 — the ~20-min static fixtured-ball capture (no robot motion): the
  taped ball on a fixture with 3+ conventional point markers, several
  heights/positions. Resolves the ABSOLUTE centroid bias — the +18 mm
  systematic disagreement between `land_err` and `arrival_dir` (plan § E-1
  standing caveat) — and decides which channel Phase 3 trusts at the plane.
- H3 — G-2: restore the hand-ODrive config from the backup JSON before any
  catch-channel work; then re-check the 0b softness verdict (it was
  measured through the braking clamp).
- H4 — G-1: the bridge-temporal latency fix (at parking that arc was at
  FW 13 committed-unflashed, S3 soak pending — read its current state, do
  not assume). The ILC learner's uptime refusal ships disabled until G-1
  fixes the healthy threshold.
- H5 — the FIRST artifact write requires `--declare-toss-cal` (the writer
  refuses unprovable provenance; the 2026-08-12 corpus never recorded its
  aim map).
- H6 — review and commit `../BallButler/ball_butler_main/hardware_config.h`
  in the BallButler repo: 11 of its 12 changed lines are pre-existing
  Jugglebot-side drift being flushed, several behavioural for BB firmware
  (`TOSS_TIER` 8b→8a, `CATCH_VEL_SCALE_DEFAULT` 0.8→0.9, ball-evidence on,
  `LEVELLING_SETTLE_S` 1.0) — review, don't rubber-stamp.

**Phase 3 itself** (the goal on return): the operator A/B per plan
§ Phase 3 — needs H1 (H2 strongly recommended first), criteria from D4,
and artifacts via H5. The absorb-or-keep decision on the aim map's update
law is made there, on evidence.

## Done-means (delete this file when)

Phase 3 has run and the absorb-or-keep decision is recorded in the plan and
logbook — or the arc is re-parked with this prompt updated in place.
