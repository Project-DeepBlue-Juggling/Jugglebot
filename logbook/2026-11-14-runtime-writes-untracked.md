---
title: "The robot stops writing tracked files: Memory and records go to temp/runtime, seeded from the tracked copies (C8 unit 1)"
type: feature
date: 2026-11-14
status: in-progress
phase: "night-shift close-out C8"
related_plan: night-shift-closeout.md
subsystem:
  - ros
  - motion
  - gui
tags:
  - safety
  - testing
files_changed:
  - ros_ws/src/jugglebot/jugglebot/runtime_paths.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/js/records-panel.js
  - .gitignore
  - tests/ros/test_runtime_writes_untracked.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_gui_records_api.py
  - logbook/INDEX.md
  - tools/runtime_records.py
  - tests/sim/test_runtime_records.py
  - .claude/commands/sitting-debrief.md
  - tests/sim/test_sitting_debrief_protocol.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - docs/records.md
---

## Summary

A running launch used to append to two tracked files in the live checkout:
`docs/records.md` and the learner's Memory `temp/learn/jugglebot/memory_v2.csv`.
The dirty tree made the 2-minute auto-pull skip for the whole Sitting. Now
`skill_node` writes gitignored copies, `temp/runtime/records.md` and
`temp/runtime/learn/<plant_id>/memory_v2.csv`. Each copy is seeded byte for byte
from its tracked file before the first load or write, and a `<file>.seed` JSON
sits beside it. This is unit 1 of Phase C8. Unit 2 adds the debrief merge
(`tools/runtime_records.py merge|status`) that copies the runtime files back.
Status stays `in-progress` until that lands and the dress rehearsal passes.

## Design

- **One path source**: `jugglebot/runtime_paths.py`. `skill_node` gets the Memory
  from `live_memory_path` and seeds the ledger with `ensure_seeded` before
  `records.record_attempt`. The launch file sets `records_path` (runtime) and
  the new `records_tracked_path` (tracked) from it. `gui_server.py` loads the
  module by path, so its top level is stdlib only. It serves the runtime
  ledger when one exists and falls back to the tracked one. That applies only
  to the default `--records-path`; an explicit path is served as given.
- **Seed** (`<runtime>.seed`): `tracked` (absolute path), `sha256` (of the
  tracked bytes at seeding, null when the tracked file was absent), `rows`
  (Memory only: `csv_data_rows`, the non-empty records after the header) and
  `seeded_at` (local ISO-8601 with offset). Both writes are atomic (temp file,
  fsync, chmod 0644, `os.replace`). The seed is written first and the copy
  second, so a crash leaves at worst a seed with no copy, which the next call
  overwrites. A crash can never leave a short Memory.
- **Missing tracked file**: the behaviour is the same as before. The Memory
  starts empty and the first append creates the file with its header. The seed
  records the absence. The schema-v1 refusal (`refuse_unmigrated_v1`, factored
  out of `Memory._load`) is now checked on the tracked directory, because the
  runtime directory never holds a v1 file.
- **Every plant_id** goes through the runtime copy, not only the git-tracked
  `jugglebot` one, so there is one rule.
- **Load log**: `_open_memory` logs one DEBUG line per goal:
  `memory loaded N rows from <runtime path> (<seeded now from ... | runtime copy kept, seeded <time> with R rows>; D dropped)`.
  It is DEBUG, not INFO, because the operator console contract
  (`TestOperatorLines`) allows one Learner line and one start line per goal at
  INFO. The INFO check is therefore the Learner's existing goal-start line
  `learner: ...; rows N (accepted ...)`, which counts the same Memory rows. A
  stale seed or a missing seed is a WARN.

## Discussion

**The stale seed** happens when the runtime copy exists but the tracked file's
SHA-256 no longer matches the seed. An auto-pull can bring rows recorded
elsewhere, someone can edit the file by hand, or a quarantine can be committed.
There were three options:

1. **Keep the runtime copy unchanged and WARN once per node (chosen).** Neither
   side is lost: the runtime rows stay on disk and the tracked rows stay in git.
   The debrief merge reconciles them. For the Memory it appends the runtime's
   post-seed rows. For records it refuses and asks.
2. **Re-merge on load** (new tracked file plus the runtime's post-seed rows).
   The robot would get the freshest learning, but this rewrites a live Memory at
   goal start. The requirement is that load never rewrites a runtime file. An
   append-merge is also wrong when the tracked edit *removed* rows, as a
   quarantine does.
3. **Prefer the tracked file and drop the runtime copy.** This silently discards
   the Sitting's rows, the exact loss the seed exists to prevent.

The accepted cost: a deliberate change to the tracked Memory (a quarantine, a
migration) no longer reaches the robot by itself. The runtime copy and its
`.seed` have to be moved aside, or the edit made to the runtime copy. The WARN
line says this. Refusing the goal on a stale seed was rejected because an
unrelated push would ground the robot mid-Sitting.

## Launch to first goal, and the second launch

1. `ros2 launch jugglebot jugglebot_launch.py`: `_repo_root()` resolves the
   checkout, and `skill_node` gets `records_path=<repo>/temp/runtime/records.md`
   and `records_tracked_path=<repo>/docs/records.md`. The launch does no file
   I/O.
2. First Juggle goal (`plant_id` `jugglebot`), after the pattern is built and
   before anything moves: `_open_memory` calls `live_memory_path(_REPO_ROOT,
   'jugglebot')`. The runtime copy is absent, so the node reads the tracked
   `temp/learn/jugglebot/memory_v2.csv` once, writes the seed (`rows` 4818 at
   origin/main today), then writes the copy. At DEBUG it logs `runtime copy ...:
   seeded now from ..., 4818 rows` and `memory loaded 4818 rows from
   .../temp/runtime/learn/jugglebot/memory_v2.csv (seeded now from ..., 4818
   rows; 0 dropped)`. At INFO the Learner line reads `learner: ...; rows 4818
   (...)`. `_make_learner` replays those bytes, which are identical
   to the tracked file, so the Learner and the first commanded landing are the
   same as before the change.
3. Each Throw's outcome goes through `Memory.append` to the runtime copy. At the
   end of the Attempt, `_offer_record` seeds `temp/runtime/records.md` if it is
   absent, then `record_attempt` writes it. The tracked files are never opened
   for writing.
4. A later goal, or a **second launch the same day**, finds the runtime copy
   present and the seed's SHA equal to the tracked file's (`kept`). It loads
   4818 + k rows, the same rows the old code would have read back from the
   tracked file. Nothing is re-copied. The tree stays clean, so the auto-pull
   keeps fast-forwarding. Only a pull that touches one of the two tracked files
   makes the seed stale (see Discussion).

## Verification

2026-11-14, `python -m pytest -q -n 2 --dist loadfile tests/ros/test_runtime_writes_untracked.py tests/ros/test_gui_records_api.py tests/ros/test_skill_node.py tests/ros/test_skill_node_robot_frame.py tests/motion/test_memory.py tests/motion/test_records.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py` after the rebase onto origin/main: 594 passed, 1 failed in 478.88 s. The one failure, `test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior`, is a pre-existing wall-clock flake (a 1.0 s mocap-staleness window against a ~0.5-1.0 s test body): it fails about 1 run in 14 on the unchanged base commit 62664f19 under box load (2026-11-14, 14 solo runs, 1 failed) and passes on rerun (`pytest -q tests/ros/test_skill_node_robot_frame.py`, 15 passed). History: an earlier scoped run (74 min, heavily loaded box) had 3 failed, 1583 passed; the TestOperatorLines start-line failure was fixed (seeding detail is DEBUG, `pytest -q tests/ros/test_skill_node.py::TestOperatorLines`: 14 passed), `test_columns_releases_each_correlate_to_their_own_track` passed 10/10 solo on both trees.

## Open Questions

- Unit 2 landed: see the Unit 2 section below.
- The dress rehearsal (launch under load): on the first goal, the INFO line
  `learner: ...; rows N (...)` must have N equal to `wc -l
  temp/learn/jugglebot/memory_v2.csv` minus 1. After that goal,
  `temp/runtime/learn/jugglebot/memory_v2.csv.seed` must exist with the same
  `rows`.
- The rendered `docs/records.md` preamble: reworded in unit 2 (below).

## Unit 2: the debrief merge

**What.** `tools/runtime_records.py merge|status` (rules and write order in
its docstring) copies the runtime ledger and every plant under
`temp/runtime/learn/*/` back into the tracked files. Per file:

| Case | merge does | exit |
|---|---|---|
| runtime absent | nothing | 0 |
| tracked == runtime | nothing (re-seeds if the seed is stale) | 0 |
| tracked sha256 == seed sha256 (null == absent) | runtime copied over tracked | 0 |
| Memory, tracked changed since the seed | runtime rows after `seed.rows` appended byte for byte onto tracked (a header if tracked is empty or absent, a newline first if its last row is torn), after proving runtime = seeded bytes + appended rows and that the headers agree | 0, else 2 |
| Memory, tracked deleted after a non-null seed | refuse | 2 |
| records.md, tracked changed since the seed | refuse with a diff; the owner answers with `--resolve-records keep-runtime` or `keep-tracked` | 2 |
| no readable seed | refuse, never guess | 2 |
| a Sitting looks live (`tools/lib/live_session.sh`) | refuse everything, list the matching processes | 3 |

`--merge-during-live-sitting-may-drop-rows` overrides the live refusal (for
BUG-85 false positives: a pytest naming a node test file matches). A refused
file does not block the others. `status` prints one line per file and always
exits 0.

**Write order** per file: journal `<runtime>.merging` (tracked sha256 before,
merged sha256), runtime := M, tracked := M, seed := seed(M), drop the journal.
Each write is `write_atomic`. A rerun reads the journal first. If the runtime
does not start with M, nothing past the journal happened and the journal is
dropped. If it does, tracked := M and the seed are redone, provided tracked is
still the pre-merge or the merged bytes (otherwise refuse). Tracked-first was
rejected: a crash before the re-seed leaves "tracked changed since seed", and
the rerun appends the same rows again. Seed-first was rejected: the new `rows`
would slice the old runtime at the wrong row. A test crashes after each of the
four steps, for the copy case and the append case, and checks that the rerun
equals the uncrashed merge; another runs a Sitting between the crash and the
rerun.

**Goal-start cost.** `ensure_seeded` now caches the present-runtime check per
`(runtime, tracked)` on the `(st_mtime_ns, st_size)` of the seed and of the
tracked file. A goal start makes three `os.stat` calls and hashes nothing
unless one of the two files changed. An empty seed already on disk (a plant
with no runtime copy yet, as in every skill_node test with a `tmp_path` root)
is no longer rewritten and fsynced at every goal start; that path reads the
small seed JSON instead. The wall-clock test that prompted this
(`test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior`)
has no tracked Memory under its `tmp_path` root, so the 40 ms hash was never
on its path; the seed write (about 4 ms, measured here) was.

**Debrief.** `sitting-debrief.md` 4.3 runs the merge, asks the owner on exit 2,
stops on exit 3, then adds `docs/records.md` and
`temp/learn/jugglebot/memory_v2.csv`. Its analysis preamble no longer calls a
dirty ledger the Sitting's expected output. `test_sitting_debrief_protocol.py`
pins the merge call.

**Records preamble.** `records.render` names the runtime copy and the merge.
The committed `docs/records.md` was re-rendered with `render(parse(text))`:
one line changed, and the result is a fixed point of the render.

### Verification (unit 2)

- 2026-11-14, `python -m pytest -q tests/sim/test_runtime_records.py`: 42
  passed in 1.54 s. Mutation check: with the journal recovery disabled, 4 of
  its tests fail.
- 2026-11-14, 12 solo runs of the wall-clock test on each tree, alternating
  base (a detached worktree of origin/main 65a53208) and this tree, under
  `flock /tmp/jugglebot-tests.lock`, load average 2.0-2.8: base 0/12 failed,
  this tree 0/12 failed (each run 3.2 s; the first two 6-8 s). Neither tree
  reproduced the failure at this load, so the measurement shows no
  difference, not an improvement.
- 2026-11-14 15:43, `flock /tmp/jugglebot-tests.lock python -m pytest -q -n 2
  --dist loadfile tests/sim/test_runtime_records.py
  tests/ros/test_runtime_writes_untracked.py tests/ros/test_gui_records_api.py
  tests/ros/test_skill_node.py tests/ros/test_skill_node_robot_frame.py
  tests/motion/test_memory.py tests/motion/test_records.py
  tests/sim/test_sitting_debrief_protocol.py tests/sim/test_logbook_index.py
  tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`: 684
  passed, 1 failed in 342.25 s. The failure is the same wall-clock test, and
  its sample was 10.36 s old against the 1.0 s window: the goal start stalled
  for about 10 s between `_frame_ready` and the offset check, which also
  covers the schedule compile. The seed check now costs three `os.stat` calls
  and the seed write about 4 ms, so neither explains 10 s. The stall is
  unexplained, and base was not run in this two-worker configuration.

### Owner commands

- Dress rehearsal (launch under load), in addition to the check above:
  `python tools/runtime_records.py status` reads `memory jugglebot ...:
  absent` before the first goal and `in step` after it, while nothing has been
  thrown.
- After a Sitting, with the launch down: `python tools/runtime_records.py
  status`, then `python tools/runtime_records.py merge`, then commit the two
  tracked files (debrief step 4.3 does this).
