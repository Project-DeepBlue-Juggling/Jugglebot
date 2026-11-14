---
title: Night Shift close-out - bind the merge to the gate, walk only the delta, stop the robot writing tracked files, notes-only Sitting sheets
created: 2026-11-13
status: active
owner: Harrison
last_updated: 2026-11-14
pathway: night-shift
branch: main
scope: >-
  The close-out phase of Pathway PWAY-9 after the first live nights: every merge to main covered by a
  gate record, a delta-only walk that exits NOTHING_NEW, skip reasons stored on the Item, runtime
  records out of the tracked tree, notes-only Sitting sheets with derived verdicts, a tested debrief
  merge script, and a Runbook that is an index.
steps:
  - {id: "Phase C1", title: Bind the merge to the gate, kind: phase, status: complete, completed: 2026-11-13, effort: m, hardware: none}
  - {id: "Phase C2", title: Runner housekeeping, kind: phase, status: complete, completed: 2026-11-14, effort: s, hardware: none}
  - {id: "Phase C3", title: Skip reasons live on the Item, kind: phase, status: complete, completed: 2026-11-14, effort: m, hardware: none}
  - {id: "Phase C4", title: Delta walk and NOTHING_NEW, kind: phase, status: complete, completed: 2026-11-14, effort: m, hardware: none}
  - {id: "Phase C5", title: Report layout and protocol, kind: phase, status: complete, completed: 2026-11-14, effort: m, hardware: none}
  - {id: "Phase C6", title: Pull stalls are loud, kind: phase, status: complete, completed: 2026-11-13, effort: s, hardware: none}
  - {id: "Phase C7", title: Offline bag reads and held-only work, kind: phase, status: complete, completed: 2026-11-14, effort: m, hardware: none}
  - {id: "Phase C8", title: The robot stops writing tracked files, kind: phase, status: in_progress, effort: m, hardware: robot}
  - {id: "Phase C9", title: Sheet notes-only and batched commits, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: "Phase C10", title: Debrief merge as a tested script, kind: phase, status: complete, completed: 2026-11-14, effort: m, hardware: none}
  - {id: "Phase C11", title: Runbook becomes an index, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: "Phase C12", title: logbook INDEX generated, kind: phase, status: complete, completed: 2026-11-14, effort: s, hardware: none}
  - {id: "Phase C13", title: Map-write check inside item commits, kind: phase, status: complete, completed: 2026-11-14, effort: s, hardware: none}
  - {id: "Phase C14", title: Night Shift report-history picker, kind: phase, status: complete, completed: 2026-11-13, effort: s, hardware: none}
  - {id: "Phase C15", title: F9 feasibility probe, kind: phase, status: complete, completed: 2026-11-14, effort: s, hardware: none}
  - {id: "Phase C16", title: Docs, kind: phase, status: complete, completed: 2026-11-14, effort: s, hardware: none}
  - {id: "Phase C17", title: R6 and close, kind: rung, status: not_started, effort: s, hardware: robot}
related_plan:
  - night-shift.md
related_logbook:
  - 2026-10-30-night-shift-design-and-plan.md
  - 2026-11-13-night-shift-closeout-plan.md
related_code:
  - tools/night_shift.sh, tools/night_shift_gate.py, tools/night_shift_preflight.py
  - mapkit/night_shift_report.py, mapkit/sitting.py, mapkit/itemcli.py, mapkit/items.py
  - .claude/commands/night-shift.md, .claude/commands/sitting-debrief.md
---

# Night Shift close-out

Branch `main`, base `origin/main` 8bc5e864 (2026-11-13). Pathway PWAY-9, the second plan under it: `night-shift.md` built and ran the Night Shift; this plan closes what the first live nights exposed. Steps C1 to C17 (ids are written `Phase C1` because the step-id grammar admits only P, R and Stage prefixes) are day-Session work: `tools/night_shift*`, `.claude/*`, `tests/sim/test_night_shift_*`, `tests/sim/test_sitting_*` and CLAUDE.md are gate-frozen for the Night Shift. The phase closes with `./run_tests.sh --full` (CLAUDE.md rule (b)) and one `/audit --unstaged`. Design source: the 2026-11-13 close-out design, decisions D1 to D10 and findings F1 to F12 cited per step.

App layout, already landed 2026-11-13 in the app repo (jugglebot-map 89f7895), not a step: the Sitting sheet progress rail, the All Items button in the frozen Items banner, the single-scroll All Items page.

## Status (2026-11-14)

- C1, C6 and C14 landed 2026-11-13. First live night under C1 (2026-11-14): `last_run.json` reads `landed: gated=0 exempt=10 ungated=0`, and the gate wrote `d64c85a5.HOLD.json` for FEAT-50. The night merged nothing, so a live MERGE record covering a merge is still to be seen. C6 is installed (`~/System_Scripts/jugglebot-pull.sh`); its status file reads OK and no live `PULL` line has been needed yet.
- R6 is met under Decision 10: BUG-80's held branch was flown at the Sitting of 2026-11-13, passed, and reached main in the debrief's push (`logbook/2026-11-13-sitting-debrief.md`; the owner merged it without the gate, see that entry's Discussion). C17 keeps the `--full` run and the archive.
- C7 landed 2026-11-14, ahead of C2, because the owner's TKT-68 answer asks the night to run the offline sweep itself. Beyond the step text: the "only under nonet" rule is enforced by a PreToolUse hook (`tools/night_shift_bash_guard.py`, read from origin/main on every Bash call, fail closed), not by deny-list spellings; the wrapper also masks `/dev/hidraw*`, `/dev/ttyTHS*` and `/dev/bus/usb`; the runner ends `NO_SETTINGS` when the hook is not wired. Gate on the landed tree (2026-11-14, `tools/night_shift_nonet.sh ./run_tests.sh -q`): parallel 601 s rc=0; serial rc=1 on one wall-clock test (`test_solver_worker_lateness`, p99 115 ms), then the serial phase rerun whole (`tools/night_shift_nonet.sh python -m pytest tests/ -q -m "serial and not nightly"`): 7 passed. The same stack before the rebase: parallel 606 s rc=0, serial 31 s rc=0, EXIT=0. Known open bypass: a script the night writes and runs unwrapped. Acceptance still to see: a live bag-read night.
- C3, C2, C4 and C5 landed 2026-11-14 (main at a690b7a0). No seeding script was run: the first night under them, 2026-11-15, is a Sunday, so it is a full walk and writes the skip reasons itself. Deviations from the step text: the committer identity is set with `GIT_COMMITTER_NAME`/`GIT_COMMITTER_EMAIL` (git has no committer-only config key); the committed `history.jsonl` row publishes `detail` only for an OK night (the repo is public); question tickets are kept out of carry, unblocked and unclassified; the Questions section ends with an `earlier: <n> ...` count line; the protocol gained a merge-conflict rule (abort, merge main into the branch, regenerate, re-gate, merge the gated tip). Gate on the landed tree (2026-11-14, `tools/night_shift_nonet.sh ./run_tests.sh -q`): parallel 614 s rc=0, serial 32 s rc=0, EXIT=0. Acceptance still to see: the first full-walk night, the first delta night and the first NOTHING_NEW night.
- C12 was landed 2026-11-14 by another Session as FEAT-52 (`logbook/INDEX.md` is generated by `python -m mapkit gen`).
- C8 is built and gated on the branch `closeout/c8-runtime-paths` (2026-11-14, `./run_tests.sh --full -q`: parallel 645 s rc=0, serial 43 s rc=0, EXIT=0; `temp/runtime` empty after the suite). It waits for the owner: merge, `colcon build`, the dress rehearsal, then a Sitting. Beyond the step text: the stale-seed check is cached on the tracked file's mtime and size, the merge keeps a journal so a crash is recovered on rerun, and tests cannot resolve a runtime path inside a real checkout.
- The gate's serial phase failed `test_solver_worker_lateness` four times in ten runs on 2026-11-14: a garbage collection in the test process (`logbook/2026-11-14-lateness-test-gc-pause.md`); fixed in the test.
- C10 (1a77568a), C13 (b23152f3) and C15 (ecdac0fd) landed 2026-11-14. C15's verdict: `IPAddressDeny` on a user scope is not enforced on this kernel (hybrid cgroups), so F9 is closed and the namespace wrapper stays. C16 landed the same day, deviating from its step text: the `last_run.json` clause in the CLAUDE.md ticker rule is kept as a conditional fallback (a stub report or the re-raise can themselves fail), and `mapkit claim`/`resolve` now run the map-write check (exit 4). C9 and C11 are built and gated on unpushed branches in both repos and wait for the owner's go-ahead, because the mapkit and app halves of C9 must deploy together outside a Sitting.

## Decisions (owner, 2026-11-13)

1. The gate is bound to the merge (C1). The walk covers the delta and exits NOTHING_NEW on an empty one; an answered ticket is walked once; the report gains a section for Items needing a small owner input (C4, C5).
2. `docs/night-shift/history.jsonl` is committed nightly, on NOTHING_NEW nights too (C2).
3. Full walk weekly plus on request (flag file). The weekday mechanism is chosen in C4: the runner sets the full walk on Sundays, the request file works any day.
4. The Night Shift may read rosbags and launch logs read-only, run `tools/probes` offline, build flown-behaviour fixes onto held branches, and edit frozen paths outside the never-list on held branches (C7). A frozen-path held branch is merged BY HAND in a day Session under the normal commit rules; no script flag exists for it.
5. Runtime records and learner memory move under `temp/runtime` (C8).
6. The Sitting sheet records notes only, with no verdict control and no override. The debrief derives verdicts and the owner confirms once. Batched commits flush on: the last step of a Launch saved, 20 minutes idle, End Sitting, app restart (C9).
7. `docs/runbook.md` shrinks to an index; the sheet is the only ordering authority; bench sheets are made discoverable in the app (C11).
8. The runner prunes merged remote `night-shift/*` branches (C2).
9. Billing is unchanged: API key primary, subscription fallback.
10. R6 (in `night-shift.md`): a merge of a held branch by the 2026-11-13 debrief on the existing prose procedure counts as meeting R6; otherwise the first merge through C10's script does. R6's status is not edited here.
11. The never-list in C7 is accepted as designed.

## 1. Steps

### Phase C1: Bind the merge to the gate

- **Invariant**: every non-exempt commit that reaches origin/main during a night is covered by a MERGE verdict record. Enforcement point: `tools/night_shift_landed.py`, called by the runner. Test: `test_night_shift_runner`, with FAKE_CLAUDE (test file :39) pushing a code commit that has no record, expecting UNGATED_MERGE, exit 1, ticker re-raised.
- **Gate** (`night_shift_gate.py` :703-736) gains `--record DIR`, atomically writing `DIR/<tip>.<VERDICT>.json` (`<tip>.MERGE.json` / `<tip>.HOLD.json`) for MERGE and HOLD alike. Fields: {schema:1, verdict, item, branch, tip, base_ref, base_sha, waived[], reasons[], gate_sha256, gate_log_sha256, audit_sha256, written_at}. DIR = env NIGHT_SHIFT_VERDICTS, default `~/.local/state/jugglebot/verdicts/`; shared with C10, so a debrief merge pushed during a night is not flagged.
- **Rule** in landed.py: each c in `rev-list --no-merges FROM..TO` has all paths (`diff-tree -r --name-only`) EXEMPT, or is an ancestor of a MERGE-record tip. Each merge m: `diff-tree --cc --name-only m` is all EXEMPT (no evil merge) and some parent of m is a MERGE-record tip.
- EXEMPT is a literal list: `map/*`, `docs/night-shift/*`, `logbook/*`, `plans/*/INDEX.md`, plus every path `mapkit gen` writes; a test pins it against gen_docs. Empty commits pass.
- **Seams**: after night_shift.sh:800 (export_posthoc_git) capture NS_ORIGIN_AFTER_CLAUDE, BEFORE deploy_step (:841), whose ROLLED_BACK reverts are runner commits. Run the check after :897. On failure finish UNGATED_MERGE: commit, author and paths in the detail, a flag in the report's Cost/Summary, and `$REPORTS/ungated.hold`; while it exists every later start ends `UNGATED_HOLD` (exit 1, a stub report each night) until the owner clears `ungated.hold`. No automatic revert.
- **Protocol**: night-shift.md 2g (:194-199) adds `--record "$NIGHT_SHIFT_VERDICTS"`; in the same commit 2e/2f (:153-190) are reordered (audit and narrative fixes first, then one full suite), otherwise the first night after landing flags every merge.
- **Prevents**: merging after a HOLD, merging without a gate run, an EXIT=0 log from another tip, a hand-edited merge commit slipping content past the gate.
- **Acceptance**: a live night with merged >= 1 and `landed: gated=k exempt=m ungated=0` in last_run.json. Risk: a 05:00 human code push is a false positive (a flag). A forged JSON record is not caught; the F1 follow-up replays the gate from the record. Rollback: revert.

### Phase C2: Runner housekeeping

- After a passing C1 check, for each `refs/remotes/origin/night-shift/*` whose tip is an ancestor of origin/main the runner (not the model) runs `git push origin --delete`, at most 20 a night; merged local branches in the clone go with `branch -d`. Names go to last_run.json `pruned[]`. Test: a temp origin with one merged and one live branch; only the merged one goes.
- `write_status` (:248) appends the last_run object to `$REPORTS/history.jsonl`. The committed `docs/night-shift/history.jsonl` is the full deduplicated local history, so a rejected push heals the next night. It rides in the cost commit on OK, the stub commit on failures, and its own commit on NOTHING_NEW/HELD/DEFERRED, only when the clone is clean on main. EXEMPT in C1 (`docs/night-shift/*`).
- The Night Shift clone commits under its own committer identity (`git config user.name`/`user.email` set in the clone by the runner at start; author unchanged). Owner, 2026-11-13. It is the marker a later change to `tools/night_shift_landed.py` can use to exempt a night-window push by the owner or a day Session, which C1 flags by design; both checkouts commit as one identity today and the `Night-Shift:` trailer is forgeable text. C2 only sets the identity and pins it with a runner test; the exemption is not built here.
- **Prevents**: eight nights of "please delete these branches", a per-night table rebuilt by hand, a dead runner that looks like an empty night.

### Phase C3: Skip reasons live on the Item

- **Invariant**: a skip reason is stored once, on the Item. Enforcement point: validation in `mapkit/items.py` (:195-222). Test: `tests/sim/test_item_night_skip.py`.
- Keys (any non-pathway Item except question tickets): `night_skip` in {sitting, input-fact, input-decision, input-sitting, owner-session, too-large, frozen-path, out-of-reach, waiting-trigger, low-priority}; `night_skip_on: YYYY-MM-DD`; `night_skip_note` (one sentence, <= 160 chars); `night_skip_input` (<= 200 chars), required iff the reason starts with `input-`. `set-status complete|dropped` removes all four.
- CLI (`itemcli.py` :705-754): `mapkit item skip <ID> --reason R --note N [--input I] | --clear`, and `--batch FILE.json [--commit]` making one commit `map: night skip reasons <date> (<n> Items)`.
- Seeding: a one-off scratchpad script maps the 11-13 Skipped rows and scope-ui-and-skips B1/B5 onto the enum; the diff is reviewed and committed once (no full-walk migration night).
- Docs: DOCUMENTATION_GUIDE frontmatter section, `docs/agents/map-writes.md`. Tier 1: the gate and preflight import `mapkit.items`, plus the app suite.
- **Prevents**: 104 copied rows a night, stale reasons carried forward (BUG-7, BUG-72).

### Phase C4: Delta walk and NOTHING_NEW

- **Invariant**: the night examines only the delta. Enforcement point: `tools/night_shift_preflight.py`. Test: table-driven cases in `test_night_shift_preflight.py`.
- delta = added + changed (`updated` or `status` differs from candidates-last.json, as `compare()` :131-140) + carry + unclassified + unblocked. Unclassified: workable, no `night_skip`, no `origin/night-shift/*-<ID>` branch. Unblocked: its `blocked_by` names an Item that changed. Question tickets enter only when added or changed, so an answered ticket is walked once and never reported again. `removed` no longer counts as new.
- candidates.json gains `delta[]` (walk order), `delta_why{id: added|changed|carry|unclassified|unblocked|answered}`, `unchanged:n`, `full_walk:bool`. Exit 3 NOTHING_NEW when the delta is empty, printing `NOTHING_NEW items=N unchanged=N`; the night's `history.jsonl` row (C2) still lands.
- **Full walk** is weekly plus on request. Mechanism: the runner sets NIGHT_SHIFT_FULL_WALK=1 itself on Sundays (`date +%u` = 7); a one-shot `$REPORTS/full-walk.request` file, deleted after the run, forces one on any day. The Sunday walk is a plain re-walk of every open Item with no skip carry-over, so it also catches the starvation case below.
- **Carry**: the post-run snapshot (:922) adds `--walked candidates-<date>.json --report docs/night-shift/<date>.md`; carry = tonight's delta minus the IDs in the report's Item rows (`parse_report`). A failed night writes no snapshot, so the next delta is taken against the older baseline and nothing is lost.
- Risk: an Item whose blocker resolves through code, not through the Item, can starve; mitigated by unblocked, carry and the Sunday full walk. Acceptance: the first live NOTHING_NEW night and the first delta night.

### Phase C5: Report layout and protocol

- **Invariant**: the layout is pinned by `mapkit/night_shift_report.py` (SECTIONS :15-26). Test: `test_night_shift_report.py`, every committed report still passes.
- New 9th section after Questions: `## Items the Night Shift could resolve with a small amount of input from you`. Columns `| Item | What it is | Input (fact/decision/sitting) | Exactly what is needed | What the night then does | Respond in |`. Respond in = the question ID filed once when the skip was set, or `sheet` for a sitting input. Rows are rendered by `python -m mapkit night inputs` from ALL open `input-*` Items (a standing, generated list, not model-written).
- Skipped lists only delta Items skipped tonight, then one last line `unchanged: <n> Items not re-examined (reasons on each Item: night_skip)`; the checker allows that line only there and accepts 8-section reports dated before the cutover.
- Protocol edits: 1.2 walks `delta`; 2c every Skipped writes `night_skip` (one batch commit), a Merge or Hold clears it; invariant 8 (:62-64) covers delta Items only; :133 an open question is not re-listed (a count line); section 3 (:356-381). `tools/night_shift_stub_report.py` writes the new section as `none recorded`; README layout updated.

### Phase C6: Pull stalls are loud

- The script's source of truth moves into the repo as `tools/systemd/jugglebot-pull.sh` (README install: `install -m 755 ... ~/System_Scripts/`); it still runs from outside the tree.
- Every run writes `temp/reports/pull/status`: `<iso> <OK|SKIPPED_DIRTY|SKIPPED_SUITE|DIVERGED|DETACHED|FETCH_FAILED> behind=<n> since=<first non-OK iso> files=<<=3>`; non-OK also via `logger -p user.warning`. Seam: today the skip at `~/System_Scripts/jugglebot-pull.sh:64-65` exits silently.
- Enforcement point: `tools/nightly_ticker.sh check` adds `PULL <status> behind=<n> since=<t>` when non-OK for more than 10 minutes; every Session runs that check at start. Test: temp repos (dirty gives SKIPPED_DIRTY with the behind count) and the ticker output.
- **Prevents**: running code 43 commits old without saying so.

### Phase C7: Offline bag reads and held-only work

**a) Bags and probes run offline.**
- Probe first, with the fallback criterion fixed in advance: `unshare --user --map-root-user --net --mount` inside nonet must bind `~/Desktop/rosbags` and `~/.ros/log` read-only (a write fails EROFS) and mask `/dev/ttyACM*`, `/dev/ttyUSB*` with `/dev/null` and `/dev/serial` with an empty tmpfs. 3/3 trials, fails closed; then `tools/night_shift_nonet.sh` gets these mounts.
- Settings: remove `Bash(*tools/probes/*)` and `Bash(*ros2 bag*)`; add denies for `ros2 bag play|record|reindex|convert`. Probes and bag reads run only under the base-extract nonet.
- Fallback if mount namespaces fail: deny each probe that imports `teensy_link`, serial or socket by name; a test greps `tools/probes` and requires every hit denied.
- Tests: `test_night_shift_settings`, plus a nonet test using temp ro/mask paths. **Prevents**: a probe's default serial or UDP target, a write into the evidence bags.

**b) Held-only work.**
- Gate `_check_c` (:472): an Item on the branch that has a `## Sitting` section HOLDs (c) "changes flown behaviour", even when rated hardware none; only the debrief's `--waive c` clears it, so merge conditions only tighten. Test: one mutant in `test_night_shift_gate`.
- Protocol 2c gets two rows: (1) a fix to flown behaviour is built, gets a `## Sitting` section with `branch:`, and is HELD; (2) a fix that needs a frozen path outside the never-list is built on a held branch as "(f) the owner merges", Respond in `none`. Per Decision 4 the owner merges (2) by hand in a day Session.
- 2e uses `D/gate-base/tools/night_shift_nonet.sh`: the extract moves before 2e and includes nonet.
- **Never-list** (not even on a branch): `tools/night_shift*`, `tools/lib/*`, `tools/systemd/*`, `tools/nightly_*`, `.claude/settings*`, `.claude/commands/night-shift.md`, the gate/guard tests, `run_tests.sh`, conftest/pytest config. `night_shift_landed.py --branches` flags any `night-shift/*` branch pushed tonight that touches it: status SELF_EDIT_BRANCH.

### Phase C8: The robot stops writing tracked files

- **Invariant**: a running launch leaves `git status` of the live checkout empty. Enforcement point: a new `jugglebot/runtime_paths.py`, the only path source for skill_node, the launch file and gui_server. Test: `tests/ros/test_runtime_writes_untracked.py` drives `Memory.append` and `records.record_attempt` through it inside a temp git repo using the repo's `.gitignore`, then asserts porcelain empty.
- Paths (Decision 5): `temp/runtime/records.md` and `temp/runtime/learn/<plant_id>/memory_v2.csv`, each with a `<file>.seed` JSON {tracked, sha256, rows (memory only), seeded_at}; on first load or write the tracked file is copied in and the seed written.
- Seams: `memory.py:217-223` (add live_memory_path, seed); `skill_node.py:4728, 4886, 5393, 5760` (Memory(...)), `:1288` and `:6912` (records_path); `jugglebot_launch.py:571`; `gui_server.py:124` and `:143-145` (read runtime, fall back to tracked); `.gitignore:101-116` (comment).
- Merge: `tools/runtime_records.py merge|status`. Runtime absent: nothing. Tracked sha equals seed sha: copy runtime over tracked. Memory changed since the seed: append runtime rows from `[seed.rows:]` (append-only file). records.md changed since the seed: refuse (exit 2), the debrief asks. After a merge the seed is rewritten, so a rerun is a no-op. Debrief 4.3 (`sitting-debrief.md:206`) calls the merge and commits both tracked files.
- **Control implication**: this changes the learner's load path. The dress rehearsal must show the first goal's "memory loaded N rows" equal to the tracked count; no planner, feedforward or timing code changes.
- Acceptance: that dress rehearsal (launch under load), then a Sitting after which the live checkout is clean and the auto-pull reports OK. Rollback: run merge, then revert.

### Phase C9: Sheet notes-only and batched commits

- Results schema 2 (`mapkit/sheet.py`; the reader still accepts 1): `verdicts[n][ID] = {notes, saved_at}` (no verdict field, no override); the debrief adds `derived[n][ID] = {verdict: pass|fail|data|inconclusive, evidence: "<log/bag path + measured value>", criterion: "<the step's serves[].pass text verbatim>", by}` and `confirmed = {at, changed[]}`. `sitting.VERDICTS` (:274) gains `data` and `inconclusive`, which closes FEAT-42.
- Debrief 1-2 (:34-98): derive verdicts against `criterion`, show one table (Item | step | criterion | measured | derived), the owner confirms or flips once; then `mapkit sheet close <name>` writes each Item's `## Sitting result` exactly once. During the Sitting the results file is the only store.
- App server: `sheets_save_step` (`app.py:516-575`) stops calling `store.update` per verdict. `GitWriter.record(path, title, batch=True)` (`gitwriter.py:249`) arms a 20-minute idle timer instead of the 5 s debounce (:63). Flush triggers (Decision 6): the step completing a Launch group, `POST /sheets/{name}/close` (End Sitting), any other API flush (`_commit` already runs `git add -A -- map`, :327-333), process restart (`_uncommitted` :193). Subject `map: Sitting sheet <name> steps a-b`.
- Parts: C9a `sheet.py` and the app server with its test (N saves in the window give one commit; a Launch boundary flushes; close flushes); C9b frontend `SheetView.tsx` shows the notes box only, after the layout work; C9c the debrief command, which starts with a close/flush call and, with C5, edits night-shift.md section 3 (the "Unaddressed Sitting results" section reads `results.yaml` files that have no `confirmed`).
- **Prevents**: 36 one-step commits a day re-triggering re-gates; the owner's live `pass`, which was wrong, missing or contradicted by the logs on 7 Sittings.

### Phase C10: Debrief merge as a tested script

- The debrief commits the Sitting's records and learner memory (C8's merge) BEFORE `prepare`, so the suite runs on a tree that already holds them. Prevents the 2026-11-13 red: a test that read `docs/records.md` failed on main only after the debrief's records commit (`logbook/2026-11-13-records-test-read-live-ledger-contents.md`; the ordering cause is inferred, not traced).
- `tools/sitting_merge.sh prepare --branch B --item ID --work DIR`: worktree; merge origin/main (generated files regenerated; a `map/` conflict stops); `./run_tests.sh --full` writes `DIR/gate-ID.log` with HEAD=, BRANCH=, EXIT=. The model then runs `/audit` and writes `DIR/audit-ID.md`.
- `tools/sitting_merge.sh land --branch B --item ID --work DIR`: base-extract gate with `--waive c --record`. MERGE: push HEAD:main, delete the remote branch, remove the worktree, append to `temp/reports/sitting-merge.jsonl`. HOLD: print the reasons and keep the branch. It has no flag for (f): a frozen-path branch cannot go through it (Decision 4).
- `FORBIDDEN_PATHS` (gate :139-169) gains `tools/sitting_merge.sh`; its test name already falls under `tests/sim/test_sitting_*`.
- Debrief 3 (:100-197) shrinks to about 25 lines; section 0 states the scratch worktree; `test_sitting_debrief_protocol` is trimmed to pointer tests. Test: temp repos with a stub suite, in the pattern of `test_night_shift_candidate.py`.

### Phase C11: Runbook becomes an index

Three phases, so neither repo breaks the other. The sheet is the only ordering authority (Decision 7).
- a) `mapkit/sitting.py`: delete `_score`/`PRIORITY_WEIGHT` (:39, :151-153), block grouping, Setup 0 and Close-out. `collect()` (:159) sorts rows by (rig, priority, ID) and keeps `blocks` as a compatibility field (one block per rig, score 0). `docs/runbook.md` becomes, per rig, `| Item | Minutes | Setup | Branch | On a sheet |`, plus `## Needs a Sitting section`, plus the header line "Fly from a Sitting sheet: /sitting-sheet robot|bench-leg|bench-hand". The `first:` key is parsed but ignored. Pins: `test_sitting_sections`; `test_runbook_close_out_points_at_debrief` now pins the /sitting-sheet line; `sheet.py:229` docstring.
- b) App: `server/nightshift.py:233-290` rows by rig. The Next Sitting sheet select is grouped by rig in optgroups, so bench sheets are discoverable; a rig with waiting Items and no sheet shows "Bench leg: n Items waiting, no sheet. In a Session: /sitting-sheet bench-leg" with a copy button.
- c) mapkit drops `blocks`; GLOSSARY entry through domain-modeling; `map-writes.md:66`.

### Phase C12: logbook INDEX generated

- `mapkit gen` writes `logbook/INDEX.md` from entry frontmatter (title, date, status, phase): date descending, then filename descending, existing header text. Currency test in `test_mapkit_gen`. Every command that hand-adds rows says "run mapkit gen" instead: log, investigate, logbook, next-phase-prompt, night-shift section 3, sitting-debrief, `logbook/README.md`. On a conflict the answer is to regenerate (`sitting_merge.sh` does this).

### Phase C13: Map-write check inside item commits

- Before committing, `mapkit item * --commit` runs the whole-map load, rated-on-leaving-inbox, the per-block readability cap and the glossary avoid-words, factored out of the four map tests into `mapkit/checks.py`. A failure refuses the commit with exit 4 and the list; a ticket with `--parent` is exempt from the similar-title refusal. Invariant 10 (night-shift.md protocol :69-79) shrinks to one line. Test: `tests/sim/test_itemcli_commit_check.py`.

### Phase C14: Night Shift report-history picker

- App repo. Today: a button per date (`NightShiftView.tsx:58-77`), state only in useState (`NightShift.tsx:28-29`), the list carries {date, path, title} (`nightshift.py:33-45`, `models.py:171`).
- Server: `list_reports` adds {merged, held, questions, inputs, status} from `parse_report` item-row counts, cached by mtime, null for unparsable files; new `GET /night-shift/runs` reads `docs/night-shift/history.jsonl`.
- UI: `<` [native select: optgroups by month, newest first; option "Fri 13 Nov . 1 merged . 2 held . 1 question", plus a status suffix on stubs] `>` and a "Latest" link off the latest. A line "Last run: <date> <status> (<delta>/<unchanged>)" from the runs list shows NOTHING_NEW nights. Default: newest report. Deep link `/projects/:id/night-shift?date=YYYY-MM-DD` (useSearchParams, push, so Back works); an unknown date shows a notice and the latest; RootRedirect (:5) forwards `search`.
- Tests: server counts; vitest for default, prev/next ends, deep link, month groups; one playwright screenshot. **Prevents**: about 365 buttons a year pushing the report below the fold, no link to one night.

### Phase C15: F9 feasibility probe

- Probe `systemd-run --user --scope -p IPAddressDeny=<a LAN test address, never 192.168.42.2>` with a Python UDP send, a setsid child and a re-exec; check `stat -fc %T /sys/fs/cgroup`. Adopt only if 3/3 sends fail with EPERM and cgroup v2 is unified; then file an L Item to replace the bridge-spelling denies. Otherwise close F9 as not feasible on 5.10-tegra and keep nonet. A logbook entry either way.

### Phase C16: Docs

- CLAUDE.md:150 ticker rule, two cases: "LOWERED by night-shift <date>: read `docs/night-shift/<date>.md` (a failed night leaves a stub and re-raises)"; "A NOTHING_NEW night does not claim: claim as usual". Drop the last_run.json clause.
- `map/pathway-night-shift.md`: D8 (API key primary, subscription fallback), Opus, the clone path, gate (a)-(h), `/sitting-debrief`.
- `night-shift.md`: R6 text ("flown from a Sitting sheet; a held branch whose debrief-derived, owner-confirmed verdict is pass, merged by `tools/sitting_merge.sh land`", with Decision 10's exception), T-U1 eight conditions, section order, `last_updated`.

### Phase C17: R6 and close

- Per Decision 10: if the 2026-11-13 debrief merges a held branch on the existing procedure, R6 is already met and this step only records it. Otherwise a Sitting flies a held branch, the debrief merges it through C10, and the plan records R6 complete with the (date, command, result) triple. Then `./run_tests.sh --full`, `/archive-plan`; PWAY-9 goes complete only on the owner's ask.

## 2. Tier, model, size, acceptance

| Step | Test tier | Model | Units | Acceptance evidence |
|---|---|---|---|---|
| C1 | tier 1 | opus: binds merges to main | 2 | fake-claude test + live night |
| C2 | tier 1 | sonnet | 1 | live night (pruned[]) |
| C3 | tier 1 + app suite | sonnet | 1+1 | map tests green after seeding |
| C4 | tier 1 | sonnet | 1 | live NOTHING_NEW night and a delta night |
| C5 | tier 1 | sonnet | 2 | live night report |
| C6 | tier 1 | sonnet | 1 | Sitting day with a PULL line |
| C7 | tier 1 | opus: hardware guard and gate | 2 | probe log, gate mutant, live bag-read Item (BUG-33/40), live held flown branch |
| C8 | tier 1, --full before the Sitting | opus: learner load path | 2 | dress rehearsal + Sitting |
| C9 | app suite + scoped | sonnet | 3 | throwaway sheet + Sitting |
| C10 | tier 1 | opus: merges to main | 2 | temp-repo tests + R6 |
| C11 | scoped + app suite | sonnet | 2.5 | currency test + app deploy |
| C12 | scoped | sonnet | 1 | currency test |
| C13 | scoped + app suite | sonnet | 1 | test |
| C14 | app suite + playwright | sonnet | 1 | deployed page |
| C15 | probe only | sonnet | 1 | logbook verdict |
| C16 | tier 3 named + /audit | sonnet | 1 | audit clean |
| C17 | --full | main session | 0.5 | R6 triple |

Each unit stays under the ~80-call agent cap. Parallel lanes (A to G in the design summary) are bounded by the files that force serial order: `tools/night_shift.sh`, `.claude/commands/night-shift.md`, `tools/night_shift_gate.py` with its test, `mapkit/night_shift_report.py`, `.claude/commands/sitting-debrief.md`, `mapkit/itemcli.py`, and the app's `server/app.py` and `server/nightshift.py`. Lane A: C1, C2, C4, C5, C7 (serial, shared runner, protocol and gate). Lane B: C3, C13 (itemcli). Lane C: C8 (ros_ws, independent). Lane D: C9, C10, C11 (sheet, debrief, sitting.py). Lane E: C6, C12, C15 (independent). Lane F: C14 (app, after C2). Lane G: C16, C17 last.

## 3. Conflicts and resolutions

1. NOTHING_NEW versus the ticker and morning-report rules: the claim happens after the preflight (`night_shift.sh:564` runs before section 9), so the ticker stays RAISED and a Session claims it. C16 states this; C2's `history.jsonl` plus C14's "Last run" line tell an empty night from a dead runner.
2. "head_before..head_after" versus the deploy tool's reverts: check origin/main as captured before `deploy_step`. Debrief merges at night write to the shared verdict directory.
3. The delta walk versus invariant 8 and README "a row per Item": both are rewritten to cover the delta (C5).
4. Frozen-path held branches versus the gate: (f) still holds them and `--waive c` cannot clear (f), so the owner merges by hand. Hazard: the branch's own nonet.sh guarding its own suite run; fixed by the base-extract nonet and the never-list scan (C7).
5. "Never merged" for flown fixes was prose: make it gate code by tightening (c); this loosens no condition.
6. Notes-only versus R6 and the report's "Unaddressed Sitting results": passed means derived and confirmed; the section reads `results.yaml` files that have no `confirmed` (C9c edits night-shift.md section 3, after C5).
7. The index Runbook versus the app /runbook, sheet scope order and a frozen debrief test: hence C11's three phases. R6 is reworded.
8. Tools that read tracked memory mid-day (migration, probes) see the state at the last debrief; acceptable, and C8's docstring says so.
9. Timing versus R6: per Decision 10, a held branch merged by the 2026-11-13 debrief on the existing prose procedure meets R6 before C10 exists; C10's first merge counts only if that did not happen. C17 is worded to match.
