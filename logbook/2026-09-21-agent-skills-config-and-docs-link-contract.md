---
title: "Agent-skill config lands, and links out of docs/ become a contract — 39 relative links (33 latent strict-build warnings) rewritten as absolute GitHub URLs"
type: refactor
date: 2026-09-21
status: resolved
phase: "docs layer — agent-skill setup"
files_changed:
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - mkdocs.yml
  - docs/agents/issue-tracker.md
  - docs/agents/triage-labels.md
  - docs/agents/domain.md
  - docs/adr/index.md
  - docs/adr/0001-offload-can-and-interpolator-from-jetson.md
  - docs/adr/0002-dedicated-second-teensy.md
  - docs/adr/0003-teensy-4_1-over-4_0.md
  - docs/adr/0004-dual-can-buses.md
  - docs/adr/0005-ethernet-over-usb-serial.md
  - docs/adr/0006-udp-not-tcp.md
  - docs/adr/0007-point-to-point-static-link.md
  - docs/adr/0008-time-sync-master-on-can-bridge.md
  - docs/adr/0009-freertos-tsandmann-port.md
  - docs/adr/0010-onboard-clock-sufficient.md
  - docs/adr/0011-mac-pinned-nmcli.md
  - docs/adr/0012-hermite-interpolator-port.md
  - docs/adr/0013-three-can-buses.md
  - docs/can_bridge/index.md
  - docs/can_bridge/control.md
  - docs/can_bridge/safety.md
  - tests/sim/test_docs_links.py
subsystem: []
tags:
  - docs
  - testing
---

# Agent-skill config lands, and "links out of docs/" become a contract

## Summary

`/setup-matt-pocock-skills` ran 2026-09-20 → 09-21. Owner's choices: GitHub Issues (`Project-DeepBlue-Juggling/Jugglebot`, `gh` at `~/bin/gh`, not on `PATH`) as the issue tracker, the default triage labels, single-context domain docs. Written: a `## Agent skills` block in CLAUDE.md and `docs/agents/{issue-tracker,triage-labels,domain}.md`; four labels (`needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`) were created on the PUBLIC GitHub repo (external state, not in git; `wontfix` already existed). Because `docs/` is published by `mkdocs build --strict`, the setup surfaced two docs-layer problems, both fixed here: `docs/agents/` would have been published, and a strict build of the branch was already red (33 warnings) before any of this.

## Motivation

1. The skill's convention puts agent config in `docs/agents/`, and MkDocs publishes everything under `docs/`.
2. Baseline strict build: 39 relative links from `docs/adr/*` and `docs/can_bridge/*` to repo files outside `docs/` (source, firmware, plans; 6 of them directory links MkDocs only logs at INFO). All 19 distinct targets exist on disk — nothing moved or went missing; the defect is link *style*, not link rot.

## Discussion

**The class, not the instance.** "A relative link that leaves `docs/`" is one root cause with 39 instances. MkDocs cannot resolve it, so strict mode warns and the published site 404s regardless. The fix is therefore a contract, all three parts together: the normative rule (Guide § 2.3, "Links out of `docs/`"), the enforcement point (`tests/sim/test_docs_links.py`, per commit), and the backstop (CI `mkdocs build --strict`).

**Why the gate had not caught it.** `.github/workflows/docs.yml` runs only on pushes to `main`/`refactor` that touch `docs/**`. Neither `origin/main` nor `origin/refactor` contains `docs/adr/`, `docs/can_bridge/`, `Teensy_code_canbridge/`, or the cited plan `plans/archived/teensy-can-offload.md` (`git ls-tree`, checked 2026-09-20 and again 2026-09-21; `origin/main` has no `plans/` at all, `origin/refactor` has other plans but not that one), and this branch is 1295 commits ahead of `origin/main`. The warnings were *latent*, not "CI was red" — CI was never observed red on them, and nothing here should be read as a red-CI history.

**Why absolute `https://github.com/…/blob/main/<path>` URLs** — each alternative leaves a named failure mode:
- `validation: links: not_found: info` silences the warnings but leaves 39 dead (404) links on the published site *and* turns the strict gate into a no-op for this whole class.
- Inline code spans instead of links: no dead links, but an ADR loses click-through to the firmware/plan it cites.
- mkdocs-macros / `{{ repo_url }}` templating: a new CI dependency (the workflow does a bare `pip install mkdocs-material`) to avoid one constant.
- Commit-SHA permalinks never 404 but freeze content — wrong for living docs (`can_bridge/safety.md` tracks the firmware).

**Tradeoff accepted.** `blob/main/…` links resolve only once this branch lands on `main`. Pages and targets travel together, so no reader sees a dead link they would not already have seen; but if docs deploy from `refactor` while the targets exist only on `main` (or vice versa), they 404 until the branches sync.

**Why a test when CI exists.** MkDocs cannot check an absolute URL. The test checks the path half exists in the tree (rename/archival rot — CLAUDE.md records plan renames silently breaking inbound links) and that no relative link escapes `docs/`; and it runs per commit, where CI runs only on push to `main`/`refactor`.

**`docs/agents/` placement.** Kept the skill's convention rather than relocating (re-runs of the setup skill look there; skills find it via the CLAUDE.md pointer). A negative control proved strict mode does *not* flag unlisted pages (INFO only), so without `exclude_docs` the three pages would have been published silently. Documented as a carve-out inside layer 3, not a ninth layer.

**Logbook → GitHub Issues?** The owner offered to replace the logbook with GitHub Issues if it simplified the mkdocs decision. Not needed: the failing links were docs → source/plan paths (a link-style problem), and the logbook/plans are pinned by tests (`test_plans_index`, `test_logbook_front_matter`) and commit trailers.

## Changes

- CLAUDE.md: `## Agent skills` block (tracker, triage labels, domain docs), each a pointer into `docs/agents/`.
- `docs/agents/{issue-tracker,triage-labels,domain}.md`: new — seed templates plus repo notes.
- `mkdocs.yml`: `exclude_docs: /agents/`.
- DOCUMENTATION_GUIDE § 2.3: the "Links out of `docs/`" rule and a "Carve-out — `docs/agents/`" paragraph.
- 17 pages (`docs/adr/0001`–`0013`, `docs/adr/index.md`, `docs/can_bridge/{index,control,safety}.md`): 39 relative links → absolute `…/Jugglebot/{blob,tree}/main/<path>` via a one-off deterministic script (asserted exactly 39 substitutions, 0 missing targets, fenced code untouched; 35 lines changed).
- `tests/sim/test_docs_links.py`: 6 tests — scanner sees the docs, no relative link leaves `docs/`, repo URLs point at existing paths, plus three detector negative controls (two for the escape half, one proving the no-rot half fails on a missing path). Pure text, no fixtures or network, xdist-safe. Existence is checked against the working tree, not git.
- External, not in git: four triage labels created on the public repo, 2026-09-21.

## Verification

- 2026-09-20, `mkdocs build --strict` (scratch venv: mkdocs 1.6.1 + mkdocs-material 9.7.7, Python 3.8) on the unmodified `docs/` + `mkdocs.yml` → exit 1, "Aborted with 33 warnings in strict mode!" (+6 INFO unrecognized relative links).
- 2026-09-20, negative control: same build with `docs/agents/` present and NO `exclude_docs` → exit 1, WARNING set identical to baseline (33 = 33, empty diff); the 3 pages appear only in the INFO "not in nav" list and ARE published (`site/agents/{domain,issue-tracker,triage-labels}`).
- 2026-09-20, after adding `exclude_docs: /agents/` → WARNING set identical to baseline; `agents/` absent from the site; ADR pages still built.
- 2026-09-21, after the 39-link rewrite, `mkdocs build --strict` → exit 0, 0 warnings, 0 unrecognized-link INFO; rewritten href present in the built HTML; `agents/` still absent.
- 2026-09-21, `pytest tests/sim/test_docs_links.py -q -p no:cacheprovider` → 5 passed in 0.06 s (before the audit added the sixth test). Detector probe: the test's scanner run over the PRE-FIX `docs/` read from `HEAD` flags 39 escaping links == MkDocs' own 39.
- 2026-09-21, `~/bin/gh label create` ×4 then `gh label list` → all five roles present (count 5).
- 2026-09-21, first gate run (before the audit-driven test edit), `./run_tests.sh -q` (phase 1 `-m "not serial and not nightly"` under xdist `--dist loadfile`, phase 2 `-m "serial and not nightly"`), run with the entry, INDEX row, docs rewrite, new test file and Guide/CLAUDE.md edits all in place → `RESULT: PASS`: parallel 279 s (rc=0), serial 18 s (rc=0), total 297 s; 6897 passed + 4 skipped, 0 failed, 0 errors. The count is derived by tallying the `-q` progress marks in `temp/logs/gate_2026-09-21_docs-link-contract.log` (pytest's own summary line is suppressed under `-q`). The 5 new tests are in that selection (`--collect-only` under the same marker expression → 5 collected). Default gate only: nothing under `controller/` or `sim/` changed, so `--full` was not required.
- 2026-09-21, `/audit --unstaged` (Sonnet reviewer over the unstaged diff plus the five untracked files): 1 WARNING + 4 NOTEs, 0 BLOCKING; the mechanical rewrite verified clean (39 links = 33 blob + 6 tree, 0 line mismatches). Applied: the stale label-setup section deleted, this entry's headline number (39 links / 33 warnings), the Guide citations, a `related_issues` disambiguation, and a control proving the test's no-rot half fails on a missing path. Declined or deferred: see Open Questions.
- Final gate, after the audit fixes: 2026-09-21, `./run_tests.sh -q` → `RESULT: PASS`: parallel 280 s (rc=0), serial 19 s (rc=0), total 299 s; 6898 passed + 4 skipped, 0 failed, 0 errors (derived from the `-q` progress marks in `temp/logs/gate_2026-09-21_post-audit.log`; +1 against the first run is the new no-rot control). This line was filled in after the run — a body-only edit, with the logbook front-matter, plans-index and docs-link tests re-run afterwards.

## Outcome

Strict docs build is clean, the agent-skill config is unpublished, and the link class is closed by rule + test + CI backstop instead of 39 patches. Audited with `/audit --unstaged` before the commit, since CLAUDE.md and DOCUMENTATION_GUIDE.md are normative docs; the commit carries `Logbook-Entry: 2026-09-21-agent-skills-config-and-docs-link-contract`.

## Open Questions

- `docs/teensy-udp-protocol.md` exists but is not in the mkdocs nav (INFO in the strict build; pre-existing; Guide § 2.3 says every page is registered). Not addressed here.
- Contract scope, from the pre-commit audit (optional; zero instances today, so nothing bites yet): (a) root-relative links such as `/plans/x.md` are not policed — MkDocs logs them at INFO only, and under the `/Jugglebot/` Pages prefix they 404, the same failure class; (b) a `blob|tree/<ref>` URL on a ref other than `main` is neither existence-checked nor rejected; (c) the no-rot check asks the working tree, not git, so a link to a gitignored path (e.g. `temp/`) would pass locally and 404 on GitHub — using `git ls-files` instead was declined because it fails spuriously on a target that exists but is not yet `git add`ed.
- A root `CONTEXT.md` (the glossary the domain skills create lazily) collides with the letter of Guide § 6 ("Don't create new `ALLCAPS.md` files for general notes") because the skill fixes the filename; whoever creates it reconciles § 6 and adds it to the § 1 table and § 4 map.
